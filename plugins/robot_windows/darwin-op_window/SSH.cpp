#include "SSH.hpp"
#include "ZIP.hpp"

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <cassert>

#include <webots/robot.h>
#include <webots/camera.h>

using namespace std;


SSH::SSH(QObject *parent) : QObject(parent) {
  mSSHSession = NULL;
  mSSHChannel = NULL;
  mSFTPChannel = NULL;
  mSFTPFile = NULL;
  mTerminate = false;
  mError = "";
}

SSH::~SSH() {
  closeSFTPChannel();
  closeSSHChannel();
  closeSSHSession();
}

int SSH::openSSHSession(const QString &IPAddress, const QString &username, const QString &password) {
  mTerminate = false;
  // Open session and set options
  mSSHSession = ssh_new();
  if (mSSHSession == NULL) {
    mError = ssh_get_error(mSSHSession);
    return -1;
  }
  ssh_options_set(mSSHSession, SSH_OPTIONS_HOST, IPAddress.toLatin1());
  ssh_options_set(mSSHSession, SSH_OPTIONS_USER, username.toLatin1());
  // Connect to server
  if (ssh_connect(mSSHSession) != SSH_OK) {
    mError = ssh_get_error(mSSHSession);
    ssh_free(mSSHSession);
    mSSHSession = NULL;
    return -1;
  }
  // Verify the server's identity
  if (verifyKnownHost() < 0) {
    mError = "Connection error: server identity not verified";
    ssh_disconnect(mSSHSession);
    ssh_free(mSSHSession);
    mSSHSession = NULL;
    return -1;
  }
  // Authenticate ourselves
  if (ssh_userauth_password(mSSHSession, NULL, password.toUtf8()) != SSH_AUTH_SUCCESS) {
    mError = ssh_get_error(mSSHSession);
    ssh_disconnect(mSSHSession);
    ssh_free(mSSHSession);
    mSSHSession = NULL;
    return -1;
  }
  return 1;
}

int SSH::verifyKnownHost() {
  int state;

  state = ssh_is_server_known(mSSHSession);
 
  switch (state) {
  case SSH_SERVER_KNOWN_OK:
    break; // ok 
  case SSH_SERVER_FOUND_OTHER:
  case SSH_SERVER_KNOWN_CHANGED:
    cerr << "DARwIn-OP SSH-RSA key has changed." << endl;
    cerr << "For security reasons, the connection will be stopped" << endl;
    cerr << "Please remove the old SSH-RSA key from the known_hosts file ("
#ifdef WIN32
            "C:\\Users\\<username>\\.ssh\\known_hosts"
#elif defined(MACOS)
            "/Users/<username>/.ssh/known_hosts"
#else // Linux
            "/home/<username>/.ssh/known_hosts"
#endif
            ")." << endl;
    return -1;
  case SSH_SERVER_FILE_NOT_FOUND: // fallback to SSH_SERVER_NOT_KNOWN
  case SSH_SERVER_NOT_KNOWN:
    if (ssh_write_knownhost(mSSHSession) < 0) {
      mError = strerror(errno);
       return -1;
    }
    break;
  case SSH_SERVER_ERROR:
    mError = ssh_get_error(mSSHSession);
    return -1;
  }
  return 0;
}

void SSH::closeSSHSession() {
  if (mSSHSession != NULL) {
    ssh_disconnect(mSSHSession);
    ssh_free(mSSHSession);
    mSSHSession = NULL;
  }
}

int SSH::openSSHChannel() {
  assert(mSSHSession);

  mSSHChannel = ssh_channel_new(mSSHSession);
  if (mSSHChannel == NULL)
    return -1;

  if (ssh_channel_open_session(mSSHChannel) != SSH_OK) {
    ssh_channel_free(mSSHChannel);
    mSSHChannel = NULL;
    return -1;
  }
  if (ssh_channel_request_shell(mSSHChannel) != SSH_OK) {
    cerr << "ssh_channel_request_shell() failed" << endl;
    closeSSHChannel();
    return -1;
  }
  return 1;
}  

void SSH::closeSSHChannel() {
  if (mSSHChannel != NULL) {
    ssh_channel_send_eof(mSSHChannel);
    ssh_channel_close(mSSHChannel);
    ssh_channel_free(mSSHChannel);
    mSSHChannel = NULL;
  }
}

void SSH::readChannel(bool display, int err) {
  char c[32];
  int i, n;
  int max = sizeof(c)-1;
  while (true) {
    n = ssh_channel_poll_timeout(mSSHChannel, 500, err);
    if (n == 0 || n == SSH_EOF || ssh_channel_is_eof(mSSHChannel) || mTerminate)
      break;
    while (true) {
      // we need to set a timeout here (1 second) to allow the mTerminate variable
      // to be checked from time to time to terminate the robot controller
      i = ssh_channel_read_timeout(mSSHChannel, c, n > max ? max : n, err, 1000);
      if (i == 0 || mTerminate) // nothing to read
        break;
      c[i] = '\0';
      QString s = QString::fromUtf8(c);
      if (display)
        emit print(s, err == 1);
      if (err)
        mStderr+=s;
      else
        mStdout+=s;
    }
    if (mTerminate)
      break;
  }
}

int SSH::executeSSHCommand(const QString &command, bool display, bool wait) {
  assert(mSSHSession);
  openSSHChannel();
  assert(mSSHChannel);
  QString cmd(command);
  ssh_channel_write(mSSHChannel, cmd.toUtf8(), cmd.size());
  if (ssh_channel_send_eof(mSSHChannel) != SSH_OK) {
    cerr << "ssh_channel_send_eof() failed" << endl;
    return -1;
  }
  mStdout = "";
  mStderr = "";
  if (wait == false)
    emit done();
  // read the return value on stderr and stdout
  while (!ssh_channel_is_eof(mSSHChannel)) {
    readChannel(display, 1); // stderr
    if (mTerminate)
      break;
    readChannel(display, 0); // stdout
    if (mTerminate)
      break;
  }
  closeSSHChannel();
  return 1;
}

int SSH::openSFTPChannel() {
  assert(mSSHSession);
  // Open SFTP channel
  mSFTPChannel = sftp_new(mSSHSession);
  if (mSFTPChannel == NULL) {
    mError = ssh_get_error(mSSHSession);
    return -1;
  }
    
  // Initilize SFTP channel
  if (sftp_init(mSFTPChannel) != SSH_OK) {
    mError = ssh_get_error(mSSHSession);
    return -1;
  }
  return 1;
}

void SSH::closeSFTPChannel() {
  if (mSFTPChannel != NULL) {
    sftp_free(mSFTPChannel);
    mSFTPChannel = NULL;
  }
}

int SSH::sendFile(const QString &source, const QString &target) {
  assert(mSSHSession && mSFTPChannel);

  int i = 0;
  int access_type = O_WRONLY | O_CREAT | O_TRUNC, nwritten;
  
  // Open local file
  FILE *file;
  file = fopen(source.toUtf8(), "rb"); 
  
  // Obtain file size:
  fseek(file, 0, SEEK_END);
  int length = ftell(file);
  rewind(file);
  
  // Allocate memory to contain the whole file:
  char *buffer = (char*)malloc(length);
  
  // Copy the file into the buffer
  int size = fread(buffer, 1, length, file);

  // Open remote file
  mSFTPFile = sftp_open(mSFTPChannel, target.toUtf8(), access_type, S_IRWXU);
  if (mSFTPFile == NULL) {
    mError = ssh_get_error(mSSHSession);
    fclose(file);
    free(buffer);
    return -1;
  }

  int maxPaquetSize = 200000;
  int packetNumber = (int)(size / maxPaquetSize);
  // Send packetNumber packet of 200kb
  for (i = 0; i < packetNumber; i++) {
    nwritten = sftp_write(mSFTPFile, (buffer + (i * maxPaquetSize)), maxPaquetSize);
    if (nwritten != maxPaquetSize) {
      mError = ssh_get_error(mSSHSession);
      sftp_close(mSFTPFile);
      fclose(file);
      free(buffer);
      return -1;
    }
  }
  // Send last packet smaller than 200kb
  nwritten = sftp_write(mSFTPFile, (buffer + (i * maxPaquetSize)), size - (packetNumber * maxPaquetSize));
  if (nwritten != (size - (packetNumber * maxPaquetSize))) {
    mError = ssh_get_error(mSSHSession);
    sftp_close(mSFTPFile);
    fclose(file);
    free(buffer);
    return -1;
  }
   
  // Close remote file
  if (sftp_close(mSFTPFile) != SSH_OK) {
    mError = ssh_get_error(mSSHSession);
    fclose(file);
    free(buffer);
    return -1;
  }

  // Close local file
  fclose(file);
  free(buffer);
  return 1;
}

int SSH::readRemoteFile(const QString &fileName, char *buffer, int buffer_size) {
  assert(mSSHSession && mSFTPChannel);

  sftp_file file;
  file = sftp_open(mSFTPChannel, fileName.toUtf8(), O_RDONLY, 0);
  if (file == NULL)
    return -1; // file not present
  int size = sftp_read(file, buffer, buffer_size-1);
  sftp_close(file);
  buffer[size] = '\0';
  return size;
}

const QString SSH::error() {
  if (mSSHSession==NULL || !mError.isEmpty())
    return mError;
  else
    return ssh_get_error(mSSHSession);
}

bool SSH::isFrameworkUpToDate() {
  int index1 = 0, index2 = 0;
  QString version, versionInstalled;
  QStringList versionList, versionInstalledList;
  QFile mFrameworkVersionFile(QProcessEnvironment::systemEnvironment().value("WEBOTS_MODULES_PATH")+"/projects/robots/darwin-op/libraries/darwin/darwin/version.txt");

  if (mFrameworkVersionFile.exists()) {
    if (mFrameworkVersionFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
      QTextStream versionStream(&mFrameworkVersionFile);

      if (!versionStream.atEnd()) {
        version = versionStream.readLine();
        versionList = version.split(".");
        index1 = versionList.at(0).toInt();
        index2 = versionList.at(1).toInt();
      }
      else
        return false; //PROBLEM file empty
    }
    else
      return false; //PROBLEM could not open file
    mFrameworkVersionFile.close();
  }
  else
    return false; //PROBLEM file do not exist


  char buffer[32];
  if (readRemoteFile("/darwin/version.txt", buffer, sizeof(buffer)) < 0) // FIXME: crashes here
    return false;

  versionInstalled = QString(buffer);
  versionInstalledList = versionInstalled.split(".");
  if (versionInstalledList.size() != 2)
    return false;

  if ( (index1 > versionInstalledList.at(0).toInt()) || (index2 > versionInstalledList.at(1).toInt()))
    return false; // new version of the Framework

  return true;
}

int SSH::updateFramework(const QString &password) {
  emit status("Updating framework");

  // ensure compatibility with the new Robotis-OP2 (the /darwin folder was renamed to /robotis)
  QString command = "if [ ! -d /darwin ]; then echo " + password + " | sudo ln -s /robotis /darwin ; fi";
  executeSSHCommand(command);

  const QString webotsModulePath = QProcessEnvironment::systemEnvironment().value("WEBOTS_MODULES_PATH");
  const QString darwinDir = webotsModulePath + "/projects/robots/darwin-op/libraries/darwin/darwin";
  const QString installArchive = QDir::tempPath() + "/webots_darwin_" + QString::number((int) QCoreApplication::applicationPid()) + "_update.zip";

  // Create archive
  ZIP::CompressFolder(installArchive, darwinDir + "/Data", true, "Data");
  ZIP::AddFolderToArchive(installArchive, darwinDir + "/Linux", true, "Linux");
  ZIP::AddFolderToArchive(installArchive, darwinDir + "/Framework", true, "Framework");
  ZIP::AddFileToArchive(installArchive, darwinDir + "/version.txt", "version.txt");

  // Send archive file
  if (sendFile(installArchive, "/darwin/update.zip") < 0) {
    // Delete local archive
    QFile deleteArchive(installArchive);
    if (deleteArchive.exists())
      deleteArchive.remove();
    return -1;
  }
  executeSSHCommand("chmod 755 /darwin/update.zip");

  // Delete local archive
  QFile deleteArchive(installArchive);
  if (deleteArchive.exists())
    deleteArchive.remove();

  // Decompress remote controller files
  executeSSHCommand("unzip /darwin/update.zip");

  // Move files and delete archive
  executeSSHCommand("cp /home/darwin/version.txt /darwin/version.txt");
  executeSSHCommand("cp -r /home/darwin/Framework /darwin");
  executeSSHCommand("cp -r /home/darwin/Linux /darwin");
  executeSSHCommand("cp -r /home/darwin/Data /darwin");
  executeSSHCommand("rm /darwin/update.zip");
  executeSSHCommand("rm /home/darwin/version.txt");
  executeSSHCommand("rm -r /home/darwin/Framework");
  executeSSHCommand("rm -r /home/darwin/Linux");
  executeSSHCommand("rm -r /home/darwin/Data");

  // Compile Framework
  executeSSHCommand("make -C /darwin/Linux/build -f Makefile clean");
  executeSSHCommand("make -C /darwin/Linux/build -f Makefile");

  // Recompile demo program
  executeSSHCommand("make -C /darwin/Linux/project/demo -f Makefile clean");
  executeSSHCommand("make -C /darwin/Linux/project/demo -f Makefile");

  // Delete version file of the Wrapper in order to force the update/recompilation of it
  executeSSHCommand("rm /darwin/Linux/project/webots/config/version.txt");

  return 1;
}

int SSH::updateFrameworkIfNeeded(const QString &password) {
  if (isFrameworkUpToDate())
    return 1;
  return updateFramework(password);
}

int SSH::updateWrapper(const QString &password) {
  emit status("Installing Webots API");

  const QString webotsModulePath = QProcessEnvironment::systemEnvironment().value("WEBOTS_MODULES_PATH");
  const QString managerDir = webotsModulePath + "/projects/robots/darwin-op/libraries/managers";
  const QString darwinDir = webotsModulePath + "/projects/robots/darwin-op";
  const QString controllerDir = webotsModulePath + "/projects/robots/darwin-op/controllers";
  const QString installArchive = QDir::tempPath() + "/webots_darwin_" + QString::number((int)QCoreApplication::applicationPid()) + "_install.zip";

  // Create archive
  emit status("Installing Webots API: Zipping files");
  ZIP::CompressFolder(installArchive, managerDir + "/include", true, "include");
  ZIP::AddFolderToArchive(installArchive, managerDir + "/src", true, "src");
  ZIP::AddFolderToArchive(installArchive, managerDir + "/lib", true, "lib");
  ZIP::AddFolderToArchive(installArchive, darwinDir + "/transfer", true, "transfer");
  ZIP::AddFolderToArchive(installArchive, darwinDir + "/config", true, "config");
  ZIP::AddFolderToArchive(installArchive, darwinDir + "/check_start_position", true, "check_start_position");
  ZIP::AddFolderToArchive(installArchive, darwinDir + "/remote_control", true, "remote_control");
  ZIP::AddFileToArchive(installArchive, controllerDir + "/Makefile.include", "Makefile.include");

  // Clean directory /darwin/Linux/project/webots
  // Remove directory webots but save controller installed
  
  emit status("Installing Webots API: Checking /darwin/Linux/project/webots/default");  
  executeSSHCommand("ls /darwin/Linux/project/webots/default >/dev/null 2>&1 && echo 1 || echo 0", false);
  if (mStdout[0] == '1')
    executeSSHCommand("mv /darwin/Linux/project/webots/default /home/darwin/default");
  emit status("Installing Webots API: Deleting previous installation if any");
  executeSSHCommand("rm -rf /darwin/Linux/project/webots");

  // Create new directory webots
  emit status("Installing Webots API: Recreating webots directory");
  if (sftp_mkdir(mSFTPChannel, "/darwin/Linux/project/webots", S_IRWXU) != 0) {
    cerr << "Problem while creating directory webots: " << ssh_get_error(mSSHSession) << endl;
    // Delete local archive
    QFile deleteArchive(installArchive);
    if (deleteArchive.exists())
      deleteArchive.remove();
    return -1;
  }
  // Create new directory controller
  if (sftp_mkdir(mSFTPChannel, "/darwin/Linux/project/webots/controllers", S_IRWXU) != 0) {
    // Delete local archive
    QFile deleteArchive(installArchive);
    if (deleteArchive.exists())
      deleteArchive.remove();
    return -1;
  }

  // Create new directory backup
  if (sftp_mkdir(mSFTPChannel, "/darwin/Linux/project/webots/backup", S_IRWXU) != 0) {
    // Delete local archive
    QFile deleteArchive(installArchive);
    if (deleteArchive.exists())
      deleteArchive.remove();
    return -1;
  }

  executeSSHCommand("ls /home/darwin/default >/dev/null 2>&1 && echo 1 || echo 0", false);
  if (mStdout[0] == '1') // Reinstall previous controller installed
    executeSSHCommand("mv /home/darwin/default /darwin/Linux/project/webots/default");

    // Send archive file
  emit status("Installing Webots API: Uploading to the robot");
  if (sendFile(installArchive, "/darwin/Linux/project/webots/install.zip") < 0) {
    // Delete local archive
    QFile deleteArchive(installArchive);
    if (deleteArchive.exists())
      deleteArchive.remove();
    return -1;
  }
  executeSSHCommand("chmod 755 /darwin/Linux/project/webots/install.zip");

  // Delete local archive
  QFile deleteArchive(installArchive);
  if (deleteArchive.exists())
    deleteArchive.remove();

  // Decompress remote controller files
  emit status("Installing Webots API: Uncompressing ZIP file on the robot");
  executeSSHCommand("unzip /darwin/Linux/project/webots/install.zip");

  // Move files and delete archive
  executeSSHCommand("mv /home/darwin/config/rc.local_original /darwin/Linux/project/webots/backup/rc.local_original");
  executeSSHCommand("mv /home/darwin/include /darwin/Linux/project/webots/include");
  executeSSHCommand("mv /home/darwin/lib /darwin/Linux/project/webots/lib");
  executeSSHCommand("mv /home/darwin/src /darwin/Linux/project/webots/src");
  executeSSHCommand("mv /home/darwin/transfer /darwin/Linux/project/webots/transfer");
  executeSSHCommand("mv /home/darwin/check_start_position /darwin/Linux/project/webots/check_start_position");
  executeSSHCommand("mv /home/darwin/config /darwin/Linux/project/webots/config");
  executeSSHCommand("mv /home/darwin/remote_control /darwin/Linux/project/webots/remote_control");
  executeSSHCommand("mv /home/darwin/Makefile.include /darwin/Linux/project/webots/controllers");
  executeSSHCommand("rm /darwin/Linux/project/webots/install.zip");

  // Compile Wrapper
  emit status("Installing Webots API: Cleaning");
  executeSSHCommand("make -C /darwin/Linux/project/webots/check_start_position -f Makefile.darwin-op clean");
  executeSSHCommand("make -C /darwin/Linux/project/webots/remote_control -f Makefile.darwin-op clean");
  executeSSHCommand("make -C /darwin/Linux/project/webots/transfer/lib -f Makefile clean");
  executeSSHCommand("make -C /darwin/Linux/project/webots/lib -f Makefile.darwin-op clean");
  emit status("Installing Webots API: Compiling \"check_start_position\"");
  executeSSHCommand("make -C /darwin/Linux/project/webots/check_start_position -f Makefile.darwin-op");
  emit status("Installing Webots API: Compiling \"remote_control\"");
  executeSSHCommand("make -C /darwin/Linux/project/webots/remote_control -f Makefile.darwin-op");
  emit status("Installing Webots API: Compiling \"transfer/lib\"");
  executeSSHCommand("make -C /darwin/Linux/project/webots/transfer/lib -f Makefile");
  emit status("Installing Webots API: Compiling \"lib\"");
  executeSSHCommand("make -C /darwin/Linux/project/webots/lib -f Makefile.darwin-op");
  // Hook demo program
  QString command = "echo "+password+" | sudo -S cp /darwin/Linux/project/webots/config/rc.local_custom /etc/rc.local";
  executeSSHCommand(command);
  executeSSHCommand("ln -s /darwin/Linux/project/demo/demo /darwin/Linux/project/webots/default");
  emit status("Installing Webots API: Done");

  return 1;
}

bool SSH::isWrapperUpToDate() {
  int index1 = 0, index2 = 0, index3 = 0;
  QString version, versionInstalled;
  QStringList versionList, versionInstalledList;
  QFile mWrapperVersionFile(QProcessEnvironment::systemEnvironment().value("WEBOTS_MODULES_PATH")+"/projects/robots/darwin-op/config/version.txt");

  if (mWrapperVersionFile.exists()) {
    if (mWrapperVersionFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
      QTextStream versionStream(&mWrapperVersionFile);

      if (!versionStream.atEnd()) {
        version = versionStream.readLine();
        versionList = version.split(".");
        index1 = versionList.at(0).toInt();
        index2 = versionList.at(1).toInt();
        index3 = versionList.at(2).toInt();
      }
      else
        return false; // file empty
    }
    else
      return false; // could not open file
    mWrapperVersionFile.close();
  }
  else
    return false; // file do not exist

  char buffer[32];
  if (readRemoteFile("/darwin/Linux/project/webots/config/version.txt", buffer, sizeof(buffer)) < 0)
    return false;

  versionInstalled = QString(buffer);
  versionInstalledList = versionInstalled.split(".");
  if ( (index1 > versionInstalledList.at(0).toInt()) || (index2 > versionInstalledList.at(1).toInt()) || (index3 > versionInstalledList.at(2).toInt()) )
    return false; // new version of the Wrapper

  return true;
}

int SSH::updateWrapperIfNeeded(const QString &password) {
  if (isWrapperUpToDate()) return 1;
  return updateWrapper(password);
}

int SSH::startRemoteCompilation(const QString &IPAddress, const QString &username, const QString &password,bool makeDefaultController) {
  if (openSSHSession(IPAddress, username, password) == -1)
    return -1;
  openSFTPChannel();
  updateFrameworkIfNeeded(password);
  updateWrapperIfNeeded(password);
  
  emit status("Starting remote compilation: Zipping controller files");
  const QString controller(wb_robot_get_controller_name());
  const QString controllerArchive(QDir::tempPath() + "/webots_darwin_" + QString::number((int)QCoreApplication::applicationPid()) + "_controller.zip");
  ZIP::CompressFolder(controllerArchive, QString(wb_robot_get_project_path()) + "/controllers/" + controller, true, controller.toUtf8());

  emit status("Starting remote compilation: Cleaning up the robot");
  executeSSHCommand("echo "+password+" | sudo -S killall -q default demo controller remote_control"); // kill default demo process (if any)
  // Clean directory controllers
  executeSSHCommand("find /darwin/Linux/project/webots/controllers/* ! -name 'Makefile.include' -delete");

  // Send archive file
  emit status("Starting remote compilation: Uploading to the robot");
  sendFile(controllerArchive, "/darwin/Linux/project/webots/controllers/controller.zip");

  executeSSHCommand("chmod 755 /darwin/Linux/project/webots/controllers/controller.zip");

  // Delete local archive
  QFile deleteArchive(controllerArchive);
  if (deleteArchive.exists())
    deleteArchive.remove();

  // Decompress remote controller files
  executeSSHCommand("unzip /darwin/Linux/project/webots/controllers/controller.zip -d /darwin/Linux/project/webots/controllers");

  // Deleting archive
  executeSSHCommand("rm /darwin/Linux/project/webots/controllers/controller.zip");

  // Compile controller
  QString makeClean = "make -C /darwin/Linux/project/webots/controllers/" + controller + " -f Makefile.darwin-op clean";
  executeSSHCommand(makeClean);
  // touch source files with the time of the robot (in order to avoid Makefile problems if time of the robot and computer are not synchronized)
  QString command = "find /darwin/Linux/project/webots/controllers/"+controller+"/* -exec touch {} \\;";
  executeSSHCommand(command);
  QString makeController = "make -C /darwin/Linux/project/webots/controllers/" + controller + " -f Makefile.darwin-op";
  executeSSHCommand(makeController);
  if (!mStderr.isEmpty()) { // return in case of compilation failure
    mError = "Controller compilation failed";
    closeSFTPChannel();
    closeSSHSession();
    return 1;
  }
  emit print("Checking ready position...\n", false);
  executeSSHCommand("/darwin/Linux/project/webots/check_start_position/check_start_position");
  if (!mStdout.startsWith("Success")) {
    emit print("Robot not in ready position!\n", true);
    emit status("Status : Robot not in ready position");
    // Remove compilation files
    QString removeController = "rm -r /darwin/Linux/project/webots/controllers/" + controller;
    executeSSHCommand(removeController);
  } else {
    // Verify that controller exist -> no compilation error
    QString controllerExist = "ls /darwin/Linux/project/webots/controllers >/dev/null 2>&1 && echo 1 || echo 0" +
            controller + "/" + controller + " | grep -c " + controller;
    executeSSHCommand(controllerExist, false);
    if (mStdout[0] == '1') { // controller exist
      if (makeDefaultController) { // setup as default controller
        executeSSHCommand("rm /darwin/Linux/project/webots/default");
        QString softLinkCommand = "cp /darwin/Linux/project/webots/controllers/" + controller + "/" + controller + " /darwin/Linux/project/webots/default";
        executeSSHCommand(softLinkCommand);
      }
      // Start controller
      executeSSHCommand("mv /darwin/Linux/project/webots/controllers/" + controller + "/" + controller + " /darwin/Linux/project/webots/controllers/" + controller + "/controller");
      executeSSHCommand("echo -e \'#!/bin/bash\\nexport DISPLAY=:0\\n/darwin/Linux/project/webots/controllers/"+controller+"/controller\\n\' > /darwin/Linux/project/webots/controllers/"+controller+"/"+controller);
      executeSSHCommand("chmod a+x /darwin/Linux/project/webots/controllers/"+controller+"/"+controller);
      executeSSHCommand("echo " + password + " | sudo -S /darwin/Linux/project/webots/controllers/"+controller+"/"+controller, true, false); // wait until we terminate it
      executeSSHCommand("echo " + password + " | sudo -S killall -q controller " + controller);
      executeSSHCommand("find /darwin/Linux/project/webots/controllers/* ! -name 'Makefile.include' -delete");
    } else  // controller do not exist
      executeSSHCommand("find /darwin/Linux/project/webots/controllers/* ! -name 'Makefile.include' -delete");
  }
  closeSFTPChannel();
  closeSSHSession();
  return 1;
}

int SSH::startRemoteControl(const QString &IPAddress, const QString &username, const QString &password) {
  if (openSSHSession(IPAddress, username, password) == -1)
    return -1;
  openSFTPChannel();
  executeSSHCommand("echo " + password + " | sudo -S killall -q remote_control default controller");
  updateFrameworkIfNeeded(password);
  updateWrapperIfNeeded(password);
  WbDeviceTag camera = wb_robot_get_device("Camera");
  int cameraWidth = wb_camera_get_width(camera);
  int cameraHeight = wb_camera_get_height(camera);
  if ((cameraWidth !=320 && cameraWidth !=160 && cameraWidth !=80 && cameraWidth !=40) ||
      (cameraHeight!=240 && cameraHeight!=120 && cameraHeight!=60 && cameraHeight!=30)) {
    QString errorString=QObject::tr("Unsupported camera resolution. Aborting...\nOnly the following resolutions are available:\n\tWidth:  320/160/80/40\n\tHeight: 240/120/60/30");
    emit print(errorString, true);
    closeSFTPChannel();
    closeSSHSession();
    return -1;
  }
  emit print("Start remote control server...\n", false);
  executeSSHCommand("echo " + password + " | sudo -S /darwin/Linux/project/webots/remote_control/remote_control "
                    + QString::number(320 / cameraWidth) + " "
                    + QString::number(240 / cameraHeight), true, false); // wait until we terminate it
  emit print("Remote control server ended\n", false);
  closeSFTPChannel();
  closeSSHSession();
  return 1;
}

int SSH::uninstall(const QString &IPAddress, const QString &username, const QString &password) {
  emit print("Uninstall...\n", false);
  if (openSSHSession(IPAddress, username, password) == -1)
    return -1;
  executeSSHCommand("ls /darwin/Linux/project/webots/backup/rc.local_original >/dev/null 2>&1 && echo 1 || echo 0", false);
  if (mStdout[0] == '1') // Restore rc.local
    executeSSHCommand("echo " + password + " | sudo -S cp /darwin/Linux/project/webots/backup/rc.local_original /etc/rc.local");
  executeSSHCommand("rm -rf /darwin/Linux/project/webots");
  closeSSHSession();
  emit print("Uninstallation successfully completed\n", false);
  return 1;
}
