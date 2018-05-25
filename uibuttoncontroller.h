#ifndef UIBUTTONCONTROLLER_H
#define UIBUTTONCONTROLLER_H

#include <QString>
#include <QObject>
#include <string>

struct buttonToggleText{
    std::string defaultButtonText = "";
    std::string toggledButtonTExt = "";
};

class uibuttonController : public QObject
{
    Q_OBJECT

public:
    //explicit uiviewController(QObject *parent =0) : QObject(parent){}
    //uiviewController();
    void togglebutton();
    void setToggleButton(std::string, std::string);
    bool toggleButtonState();
    QString getButtonText();

    bool buttonstate = true;
    buttonToggleText btntext;
    std::string buttonText = "";

signals:
    void updateButtonText(QString newbttext);

};



#endif // UIBUTTONCONTROLLER_H
