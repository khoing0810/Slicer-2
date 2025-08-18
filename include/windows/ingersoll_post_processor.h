#pragma once

#if 0

    #include "QGridLayout"
    #include "QIcon"
    #include "QLabel"
    #include "QPushButton"
    #include "QStatusBar"
    #include "QWidget"

namespace ORNL
{
    /*!
     * \class IngersollPostProcessor
     * \brief Tool Ingersoll Post-Processor Window
     */
    class IngersollPostProcessor : public QWidget
    {
    public:
        //! \brief Constructor
        IngersollPostProcessor(QWidget* parent);

        //! \brief Destructor
        ~IngersollPostProcessor();
    signals:

    private slots:
        //! \brief Open a file dialog to choose a file to be post-processed
        void openFile2Process();

    protected:
        QWidget *parentWindow;
        void closeEvent(QCloseEvent *event);

    private:
        QString savedPath;
        QGridLayout *m_layout;
        QStatusBar *m_statusbar;
        QPushButton *m_openFile2Process;

        void setupEvents();
        void processFile();
        bool processFile(QString input, QString output);
    };
}

#endif
