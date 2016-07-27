/*
 * Author: Adam Allevato
 * Date: 2015-07-31
 */

#include "step.h"
#include "ui_step.h"

#include <sstream>
#include <iostream>
#include <cstdio>

#include <QScrollArea>

bool isOnlyDouble(const char* str)
{
    char* endptr = 0;
    strtod(str, &endptr);

    if(*endptr != '\0' || endptr == str)
        return false;
    return true;
}

Step::Step(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Step)
{
    ui->setupUi(this);
}

Step::~Step()
{
    delete ui;
}

std::string Step::getAction() {
   QComboBox* action_combo = this->findChild<QComboBox*>("action_combo");
   return action_combo->currentText().toStdString();
}

std::string Step::getActionText() {
    QLineEdit* action_lineEdit = this->findChild<QLineEdit*>("action_lineEdit");
    return action_lineEdit->text().toStdString();
}

std::string Step::getCondition() {
    QComboBox* condition_combo = this->findChild<QComboBox*>("condition_combo");
    return condition_combo->currentText().toStdString();
}

std::string Step::getConditionText() {
    QLineEdit* condition_lineEdit = this->findChild<QLineEdit*>("condition_lineEdit");
    return condition_lineEdit->text().toStdString();
}

std::string Step::produceYAML() {
    std::stringstream cumulative;

    cumulative      << "  - action: " << getAction() << std::endl;
    if(getAction() != "none" && getAction() != "completion") {
        cumulative  << "    action_text: " << getActionText() << std::endl;
    }
    cumulative      << "    condition: " << getCondition() << std::endl;
    cumulative      << "    condition_text: " << getConditionText() << std::endl;

    QCheckBox* required_checkBox = this->findChild<QCheckBox*>("required_checkBox");
    if(required_checkBox->isChecked()) {
        cumulative  << "    required: true" << std::endl;
    }

    QCheckBox* temporary_checkBox = this->findChild<QCheckBox*>("temporary_checkBox");
    if(temporary_checkBox->isChecked()) {
        cumulative  << "    temporary: true" << std::endl;
    }

    return cumulative.str();
}

void Step::on_pushButton_clicked()
{
    delete this;
}

void Step::moveUp()
{
    QVBoxLayout* layout = QWidget::window ()->findChild<QVBoxLayout*>("stepsLayout");
    int index = layout->indexOf(this);
    int newIndex = index - 1;    if(newIndex < 0) newIndex = 0;
    if(newIndex >= layout->count()) newIndex = layout->count() - 1;

    layout->removeWidget(this);
    layout->insertWidget(newIndex, this);

    refreshUpDownButtons(layout);
}

bool Step::validate()
{
    bool valid = true;

    if(getAction() != "none" && getActionText().empty()) {
        this->findChild<QLineEdit*>("action_lineEdit")->setStyleSheet("background-color: pink;");
        valid = false;
    } else {
        this->findChild<QLineEdit*>("action_lineEdit")->setStyleSheet("");
    }

    if(getCondition() != "none" && getCondition() != "completion" && getConditionText().empty()) {
        this->findChild<QLineEdit*>("condition_lineEdit")->setStyleSheet("background-color: pink;");
        valid = false;
    } else if(getCondition() == "seconds" && !isOnlyDouble(getConditionText().c_str())) {
        this->findChild<QLineEdit*>("condition_lineEdit")->setStyleSheet("background-color: pink;");
        valid = false;
    } else {
        this->findChild<QLineEdit*>("condition_lineEdit")->setStyleSheet("");
    }

    return valid;
}

void Step::moveDown()
{
    QVBoxLayout* layout = QWidget::window ()->findChild<QVBoxLayout*>("stepsLayout");
    int index = layout->indexOf(this);
    int newIndex = index + 1;
    if(newIndex < 0) newIndex = 0;
    if(newIndex >= layout->count()) newIndex = layout->count() - 1;

    layout->removeWidget(this);
    layout->insertWidget(newIndex, this);

    refreshUpDownButtons(layout);
}

void Step::refreshUpDownButtons(QVBoxLayout* layout)
{
    QList<Step*> widgets =  QWidget::window ()->findChild<QScrollArea*>("scrollArea")->findChildren<Step*>();
    if(widgets.length() < 1) {
        std::cout << "no children: " << layout->count() << std::endl;
        return;
    }
    for(int i = 0; i < layout->count(); ++i) {
        static_cast<Step*>(layout->itemAt(i)->widget())->checkUpDownButtons(layout);
    }
}

void Step::checkUpDownButtons(QVBoxLayout* layout)
{
    int index = layout->indexOf(this);
    if(index <= 0) {
        this->findChild<QPushButton*>("upButton")->setDisabled(true);
    } else {
        this->findChild<QPushButton*>("upButton")->setDisabled(false);
    }

    if(index+1 == layout->count()) {
        this->findChild<QPushButton*>("downButton")->setDisabled(true);
    } else {
        this->findChild<QPushButton*>("downButton")->setDisabled(false);
    }
}

void Step::on_action_combo_currentTextChanged(const QString &arg1)
{
    if(arg1.toStdString() == "none") {
        this->findChild<QLineEdit*>("action_lineEdit")->setDisabled(true);
    } else {
        this->findChild<QLineEdit*>("action_lineEdit")->setDisabled(false);
    }
}

void Step::on_condition_combo_currentTextChanged(const QString &arg1)
{
    if(arg1.toStdString() == "none" || arg1.toStdString() == "completion") {
        this->findChild<QLineEdit*>("condition_lineEdit")->setDisabled(true);
    } else {
        this->findChild<QLineEdit*>("condition_lineEdit")->setDisabled(false);
    }
}

void Step::on_upButton_clicked()
{
    moveUp();
}

void Step::on_downButton_clicked()
{
    moveDown();
}
