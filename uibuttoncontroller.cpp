#include "uibuttoncontroller.h"

//uibuttonController::uibuttonController()
//{

//}

void uibuttonController::setToggleButton(std::string defaultText, std::string toggledText){
    btntext.defaultButtonText = defaultText;
    btntext.toggledButtonTExt = toggledText;
}

void uibuttonController::togglebutton(){
    if(!buttonstate){
        buttonText = btntext.defaultButtonText;
        buttonstate = true;
    }
    else{
        buttonText = btntext.toggledButtonTExt;
        buttonstate = false;
    }
    emit updateButtonText(getButtonText());
}

bool uibuttonController::toggleButtonState(){
    return buttonstate;
}

QString uibuttonController::getButtonText(){
    QString Qbuttontext = QString::fromStdString(buttonText);
    return Qbuttontext;
}


