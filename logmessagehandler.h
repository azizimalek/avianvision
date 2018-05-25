#ifndef LOGMESSAGEHANDLER_H
#define LOGMESSAGEHANDLER_H

#include <QObject>
#include <thread>
#include <chrono>
#include <QString>

struct logMessageInterface{
    std::string message;
    bool new_message;
};

class logmessagehandler : public QObject
{
    Q_OBJECT

public:
    logmessagehandler();
    void startLogging();
    void logging();
    void warning(std::string);
    void error(std::string);
    void status(std::string);
    void enableLogDisplay(){enableLogDisplaybool = true;}
    void disableLogDisplay(){enableLogDisplaybool = false;}
    void logEmitter(std::string);

    bool new_message = false;

private:
    std::string mlogMessage;
    bool enableLogDisplaybool = true;

signals:
    void messageReceived(QString logmessage);

};

#endif // LOGMESSAGEHANDLER_H
