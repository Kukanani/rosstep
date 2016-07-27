/*
 * Author: Adam Allevato
 * Date: 2015-07-31
 */

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "step.h"
#include "ui_step.h"

#include "QFileDialog"

#include "stdlib.h"
#include <cstdio>
#include <iostream>
#include <sstream>
#include <fstream>

#include "yaml-cpp/yaml.h"

bool replace(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    addStep();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::addStep()
{
    Step* s = new Step();
    ui->stepsLayout->insertWidget(0, s);

    s->refreshUpDownButtons(ui->stepsLayout);
}

void MainWindow::on_add_button_clicked()
{
    addStep();
}

void MainWindow::save() {
    std::string cumulative;
    const QList<Step*> widgets = ui->scrollArea->findChildren<Step*>();
    if(widgets.length() < 1) {
        std::cout << "no items in the list" << std::endl;
        return;
    }
    bool valid = true;
    for(int i = 0; i < ui->stepsLayout->count(); ++i) {
        valid &= static_cast<Step*>(ui->stepsLayout->itemAt(i)->widget())->validate();
    }
    if(!valid) {
        return;
    }
    for(int i = 0; i < ui->stepsLayout->count(); ++i) {
        cumulative += static_cast<Step*>(ui->stepsLayout->itemAt(i)->widget())->produceYAML();
    }
    std::cout << cumulative << std::endl;

    QFileDialog* qfd = new QFileDialog(this, Qt::WindowFlags(Qt::WindowModal));
    qfd->setDirectory(qfd->directory().path() + "/output");
    QUrl qurl = qfd->getSaveFileUrl();
    std::string url = qurl.path().toStdString();
    std::string file = qurl.fileName().toStdString();
    if(file.find(".yaml") == std::string::npos) {
        file += ".yaml";
    }
    int pos = url.find_last_of("/")+1;
    std::string folder = url.substr(0, pos);

    std::string yamlPath = folder + file;

    std::cout << yamlPath << std::endl;

    //output the rosstep yaml
    std::ofstream ofstr(yamlPath.c_str(), std::ofstream::out);
    YAML::Emitter out(ofstr);
    ofstr << cumulative;
    ofstr.close();

    //make the .desktop file
    std::string desktopFile = file;
    replace(desktopFile, "yaml", "desktop");
    std::string desktopPath = folder + desktopFile;
    std::cout << desktopPath << std::endl;
    std::ofstream desktop_stream(desktopPath.c_str(), std::ofstream::out);

    std::string name = file;
    replace(name, ".yaml", "");
    std::string runScript = std::string("bash -i -c \"rosrun rosstep rosstep.py ") + yamlPath + "\"";

    desktop_stream << "[Desktop Entry]" << std::endl;
    desktop_stream << "Version=1" << std::endl;
    desktop_stream << "Name=Run " << name << std::endl;
    desktop_stream << "Comment=Command: " << url << std::endl;
    desktop_stream << "Exec=" << runScript << std::endl;
    desktop_stream << "Icon=" << QCoreApplication::applicationDirPath().toStdString() << "/step.png" << std::endl;
    desktop_stream << "Terminal=true" << std::endl
                   << "Type=Application" << std::endl
                   << "Categories=Application;" << std::endl;


    desktop_stream.close();
    std::string command = "chmod 755 " + desktopPath;
    system(command.c_str());
}
