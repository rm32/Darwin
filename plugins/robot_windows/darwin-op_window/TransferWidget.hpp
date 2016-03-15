/*
 * Description:  Widget for communicating with the robot DARwIn-OP
 */

#ifndef TRANSFER_WIDGET_HPP
#define TRANSFER_WIDGET_HPP

#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include <QtWidgets/QtWidgets>

class SSH;

class TransferWidget : public QWidget
{
  Q_OBJECT

  public:
    enum Status {
      DISCONNECTED,
      START_REMOTE_CONTROL,
      RUN_REMOTE_CONTROL,
      STOP_REMOTE_CONTROL,
      UNINSTALL,
      START_REMOTE_COMPILATION,
      RUN_REMOTE_COMPILATION,
      STOP_REMOTE_COMPILATION
    };

                        TransferWidget(QWidget *parent = 0);
    virtual            ~TransferWidget();

    // void                robotInstableSlot(); // TODO: this function is never call: when has it been unused?

  public slots:
    void                makeDefaultControllerWarning(int s);
    void                restoreSettings();
    void                uninstall();
    void                sendController();
    void                remoteControl();
    void                SSHSessionComplete();
    void                SSHSessionDone();
    void                print(const QString &message, bool error);
    void                status(const QString &message);

  /*
  signals:
    void                setStabilityResponseSignal(int stability);
    void                isRobotStableSignal();
  */

  private:
    void                saveSettings();
    void                enableButtons();
    void                disableButtons();

    void                showProgressBox(const QString &title,const QString &message);
    void                finishStartRemoteCompilation();
    void                finishStartRemoteControl();
    void                finish();
    void                loadSettings();
    
    // reimplement StandardPaths::getCurrentLibraryPath to retrieve path of this robot window library
    QString             getCurrentLibraryPath();

    QFutureWatcher<int> mFutureWatcher;
    QFuture<int>        mFuture;

    Status              mStatus;

    //***  SSH  ***//
    SSH                *mSSH;
    bool                mConnectionState;

    QGridLayout        *mContainerGridLayout;

    //***  SETTINGS  ***//
    QGridLayout        *mSettingsGridLayout;
    QGroupBox          *mSettingsGroupBox;

    // IP adresse
    QLineEdit          *mIPAddressLineEdit;
    QLabel             *mIPLabel;

    // Username
    QLineEdit          *mUsernameLineEdit;
    QLabel             *mUsernameLabel;

    // Password
    QLineEdit          *mPasswordLineEdit;
    QLabel             *mPasswordLabel;

    // Restore
    QPushButton        *mDefaultSettingsButton;
    QSettings          *mSettings;

    //***  Upload controller  ***//
    QGridLayout        *mActionGridLayout;
    QGroupBox          *mActionGroupBox;

    // Controller
    QIcon              *mSendControllerIcon;
    QIcon              *mStopControllerIcon;
    QPushButton        *mSendControllerButton;
    QCheckBox          *mMakeDefaultControllerCheckBox;

    // remote control
    QIcon              *mRemoteControlIcon;
    QPushButton        *mRemoteControlButton;
    bool                mRemoteEnable;

    // Remote control wait during starting
    QProgressDialog    *mRemoteProgressDialog;
    QProgressBar       *mRemoteProgressBar;

    // Wrapper
    QPushButton        *mUninstallButton;

    //***  OUTPUT  ***//
    QGridLayout        *mOutputGridLayout;
    QGroupBox          *mOutputGroupBox;

    // Status label and progress bar
    QLabel             *mStatusLabel;
    QProgressBar       *mProgressBar;

    // Console Show
    QTextEdit          *mConsoleShowTextEdit;
};

#endif
