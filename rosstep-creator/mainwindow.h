/*
 * Author: Adam Allevato
 * Date: 2015-07-31
 */
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

//forward defines
namespace Ui {
class MainWindow;
}
class QVBoxLayout;

/**
 * @brief Main window for the ROSStep creator.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    /**
     * @brief add a single step to the stack, at the end of the list
     */
    void addStep();

public slots:

    void save();

private slots:
    void on_add_button_clicked();
private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
