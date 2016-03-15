#ifndef ZIP_HPP
#define ZIP_HPP

class QString;
struct zip;

class ZIP {
  public:
                ZIP();
    virtual    ~ZIP();
    
    // Compress a directory and all the sub-directory
    // archiveName : Name of the desired archive
    // folder      : folder to compress
    // firstFolder : All the file and directory are located in a directory in the archive
    // recursive   : Include sub-directory
    static bool CompressFolder(QString archiveName, QString folder, bool recursive = true, const char * firstFolder = "");
    
    // Add a directory and all the sub-directory to an archive
    // archiveName : Name of the desired archive
    // folder      : folder to compress
    // firstFolder : All the file and directory are located in a directory in the archive
    // recursive   : Include sub-directory
    static bool AddFolderToArchive(QString archiveName, QString folder, bool recursive = true, const char * firstFolder = "");
    
    static bool AddFileToArchive(QString archiveName, QString file, const char * nameInArchive = "");

  private:
    static bool AddFolder(struct zip *archive, QString folder, QString name, bool recursive = true);
    static bool AddFile(struct zip *archive, QString file, QString name);
};

#endif
