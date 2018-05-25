#include "logmessagehandler.h"

logMessageInterface logger;

logmessagehandler::logmessagehandler()
{

}

void logmessagehandler::startLogging(){
    std::thread log_thread (&logmessagehandler::logging, this);
    log_thread.detach();
}

void logmessagehandler::logging(){
    while(1){
//        if(new_message)
//            if(enableLogDisplaybool)
//                emit messageReceived(QString::fromStdString(mlogMessage));

        if(logger.new_message){
            if(enableLogDisplaybool){
                emit messageReceived(QString::fromStdString(logger.message));
            }

            new_message = false;
            logger.new_message = false;
        }
         std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void logmessagehandler::logEmitter(std::string message){
    new_message = true;
    logger.new_message = new_message;
    logger.message = mlogMessage;
    //emit messageReceived(QString::fromStdString(message));
}

void logmessagehandler::warning(std::string logmessage){
    mlogMessage = "[WARNING]:" + logmessage;
    logEmitter(mlogMessage);
}

void logmessagehandler::error(std::string logmessage){
    mlogMessage = "[ERROR]:" + logmessage;
    logEmitter(mlogMessage);
}

void logmessagehandler::status(std::string logmessage){
    mlogMessage = "[STATUS]:" + logmessage;
    logEmitter(mlogMessage);
}
