// enviroment variables recognized:
//
// BAUDRATE=xxxx set the baudrate on the serial port, note the camera only recognizes a few values
// DEBUGSERIAL if set to any value will print the command packets and replies to stderr

#include "c328.h"

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>

// For compilers that support precompilation, includes "wx/wx.h".
#include "wx/wxprec.h"


#ifndef WX_PRECOMP
    #include "wx/wx.h"
#endif

#include "wx/image.h"
#include "wx/mstream.h"

using namespace std;

class MyApp : public wxApp
{
public:
    virtual bool OnInit();
};

class MyFrame : public wxFrame
{
public:
    MyFrame();

    void OnPaint(wxPaintEvent& event);
    void OnTimer(wxTimerEvent& event);

private:
    
    void get_image();
    void load_image(uint8_t* data, size_t datasz);
    void OnHello(wxCommandEvent& event);
    void OnAbout(wxCommandEvent& event);

    C328* _dev;
    wxBitmap _cmyk_jpeg;
    wxTimer _timer;
    
    wxDECLARE_DYNAMIC_CLASS(MyFrame);
    wxDECLARE_EVENT_TABLE();
};

enum
{
    ID_Hello = 1,
    ID_Timer = 2
};

// main function
wxIMPLEMENT_APP(MyApp);

bool MyApp::OnInit()
{
    wxInitAllImageHandlers();

    MyFrame *frame = new MyFrame();
    frame->Show(true);

    return true;
}

MyFrame::MyFrame()
    : wxFrame(nullptr, wxID_ANY, "camera_display"), _dev(nullptr), _timer(this, ID_Timer)
{
    wxMenu *menuFile = new wxMenu;
    menuFile->Append(ID_Hello, "&Hello...\tCtrl-H",
                     "Help string shown in status bar for this menu item");
    menuFile->AppendSeparator();
    menuFile->Append(wxID_EXIT);
    wxMenu *menuHelp = new wxMenu;
    menuHelp->Append(wxID_ABOUT);
    wxMenuBar *menuBar = new wxMenuBar;
    menuBar->Append(menuFile, "&File");
    menuBar->Append(menuHelp, "&Help");
    SetMenuBar(menuBar);
    auto statusBar = CreateStatusBar();
    SetStatusText("Welcome to wxWidgets!");

    Bind(wxEVT_MENU, &MyFrame::OnHello, this, ID_Hello);
    Bind(wxEVT_MENU, &MyFrame::OnAbout, this, wxID_ABOUT);
    Bind(wxEVT_MENU, [=](wxCommandEvent&) { Close(true); }, wxID_EXIT);

    std::string baudrate("9600");
    
    // check environment for baudrate
    char* envval = getenv("BAUDRATE");
    if (envval != nullptr)
        baudrate = envval;

    _dev = new C328("/dev/ttyUSB0", baudrate);

    // check environment for debug print
    envval = getenv("DEBUGSERIAL");
    if (envval != nullptr) {
        SetStatusText("Turning serial command packet printing on");
        _dev->set_debug(true);
    }

    _dev->sync();
    _timer.Start(10000);
    //get_image();

    SetClientSize(320,200+menuBar->m_height+statusBar->m_height);

    Show();
}

void MyFrame::get_image()
{
    //load_image(nullptr, 0);
    //return;
    if (_dev->is_connected())
    {
        std::cerr << "Connected to camera\n";
        bool retval = false;
        for (int i=0; i<5 && !retval; i++)
        {
            retval = _dev->initial(TwoBitGrayScale, P160x120, J320x240);
        }
        if (!retval)
        {
            _dev->reset(true);
            std::cerr << "not initialized" << std::endl;
            return;
        }
        std::cerr << "initial setup done\n";
        retval = false;
        for (int i=0; i<5 && !retval; i++)
        {
            retval = _dev->set_pkg_size();
        }
        if (!retval)
        {
            _dev->reset(true);
            std::cerr << "set pkg size failed" << std::endl;
            return;
        }
        std::cerr << "set pkg size done\n";
        auto result = _dev->jpeg_snapshot();
        if (result.first != 0)
        {
            load_image(result.second, result.first);
            delete [] result.second;
        }
    }
    else
        _dev->reset(true);
    Fit();
}

void MyFrame::load_image(uint8_t* data, size_t datasz)
{
    wxMemoryInputStream idstream(data, datasz);
    
    wxImage image;
    if (!image.LoadFile(idstream, wxBITMAP_TYPE_JPEG)) {
//    if (!image.LoadFile("/spare/export/src/boat-os/build-camera-display/pix00.jpg")) {
        cerr << "Can't load image from data" << endl;
        return;
    }
    _cmyk_jpeg = wxBitmap(image);
}

void MyFrame::OnTimer(wxTimerEvent& WXUNUSED(event))
{
    get_image();
}

void MyFrame::OnPaint(wxPaintEvent& WXUNUSED(event))
{
    wxPaintDC dc(this);
    PrepareDC(dc);
    if (_cmyk_jpeg.IsOk())
        dc.DrawBitmap(_cmyk_jpeg, 0, 0);
}
    
void MyFrame::OnAbout(wxCommandEvent& event)
{
    wxMessageBox("This is a wxWidgets Hello World example",
                 "About Hello World", wxOK | wxICON_INFORMATION);
}

void MyFrame::OnHello(wxCommandEvent& event)
{
    wxLogMessage("Hello world from wxWidgets!");
}

IMPLEMENT_DYNAMIC_CLASS(MyFrame, wxFrame)
wxBEGIN_EVENT_TABLE(MyFrame, wxFrame)
    EVT_PAINT(MyFrame::OnPaint)
    EVT_TIMER(ID_Timer, MyFrame::OnTimer)
wxEND_EVENT_TABLE()
