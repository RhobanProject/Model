#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <ncurses.h>
#include <signal.h>

//CODE
#include "Types/VectorLabel.hpp"
#include "CartWalk/CartWalkProxy.hpp"

//UTILS
#include "SDKConnection.hpp"

void drawTitleWin(WINDOW* titleWin)
{
    wborder(titleWin, ACS_VLINE, ACS_VLINE, ACS_HLINE, ACS_HLINE, '+', '+', '+', '+');
    wattrset(titleWin, A_BOLD | A_UNDERLINE);
    mvwprintw(titleWin, 1, 1, "CartWalk Tunner (NCurses)");
    wattrset(titleWin, A_NORMAL);
    mvwprintw(titleWin, 3, 1, "q:      quit");
    mvwprintw(titleWin, 4, 1, "i:      init position");
    mvwprintw(titleWin, 5, 1, "space:  toggle enabled");
    mvwprintw(titleWin, 6, 1, "escape: emergency");
    wrefresh(titleWin);
}

void drawStatusWin(WINDOW* statusWin, bool isEnabled, double delta)
{
    wclear(statusWin);
    wborder(statusWin, ACS_VLINE, ACS_VLINE, ACS_HLINE, ACS_HLINE, '+', '+', '+', '+');
    wattrset(statusWin, A_BOLD);
    mvwprintw(statusWin, 1, 1, "Status");
    wattrset(statusWin, A_NORMAL);
    if (isEnabled) {
        mvwprintw(statusWin, 3, 1, "Enabled: TRUE");
    } else {
        mvwprintw(statusWin, 3, 1, "Enabled: FALSE");
    }
    mvwprintw(statusWin, 4, 1, "Delta: %0.3f", delta);
    wrefresh(statusWin);
}

void drawParamsWin(WINDOW* paramsWin, 
    const Leph::VectorLabel& staticParams, 
    const Leph::VectorLabel& dynamicParams,
    int selected)
{
    wclear(paramsWin);
    wborder(paramsWin, '|', '|', '-', '-', '#', '#', '#', '#');
    wattrset(paramsWin, A_BOLD);
    mvwprintw(paramsWin, 1, 1, "Dynamic Parameters");
    wattrset(paramsWin, A_NORMAL);
    int line = 3;
    for (size_t i=0;i<dynamicParams.size();i++) {
        if (i == selected) {
            wattrset(paramsWin, A_BOLD);
            std::string label = "# " + dynamicParams.getLabel(i);
            mvwprintw(paramsWin, line, 2, label.c_str());
            mvwprintw(paramsWin, line, 20, "%.3f", dynamicParams(i));
            wattrset(paramsWin, A_NORMAL);
        } else {
            std::string label = "- " + dynamicParams.getLabel(i);
            mvwprintw(paramsWin, line, 2, label.c_str());
            mvwprintw(paramsWin, line, 20, "%.3f", dynamicParams(i));
        }
        line++;
    }
    line++;
    wattrset(paramsWin, A_BOLD);
    mvwprintw(paramsWin, line, 1, "Static Parameters");
    wattrset(paramsWin, A_NORMAL);
    line += 2;
    for (size_t i=0;i<staticParams.size();i++) {
        if (i + dynamicParams.size() == selected) {
            wattrset(paramsWin, A_BOLD);
            std::string label = "# " + staticParams.getLabel(i);
            mvwprintw(paramsWin, line, 2, label.c_str());
            mvwprintw(paramsWin, line, 20, "%.3f", staticParams(i));
            wattrset(paramsWin, A_NORMAL);
        } else {
            std::string label = "- " + staticParams.getLabel(i);
            mvwprintw(paramsWin, line, 2, label.c_str());
            mvwprintw(paramsWin, line, 20, "%.3f", staticParams(i));
        }
        line++;
    }
    wrefresh(paramsWin);
}

int main()
{
    //Initialize the connection
    Leph::SDKConnection sdkConnection;

    //Initialisation of CartWalk parameters
    Leph::CartWalkProxy walk;
    Leph::VectorLabel staticParams = walk.buildStaticParams();
    Leph::VectorLabel dynamicParams = walk.buildDynamicParams();

    //Ncurses initialization
    initscr();
    //Enable special input keys
    keypad(stdscr, TRUE);
    //Disable input echo
    noecho();
    //Disable input buffering
    //(and catches Ctrl-Z and Ctrl-C)
    raw();
    //Enable non-blocking input reading
    nodelay(stdscr, TRUE);
    //Hide cursor
    curs_set(0);

    move(0, COLS/2-35/2);
    refresh();

    WINDOW* titleWin = newwin(8, 38, 1, 1);
    drawTitleWin(titleWin);
    
    WINDOW* statusWin = newwin(6, 38, 10, 1);
    drawStatusWin(statusWin, dynamicParams("enabled"), 0.2);
    
    WINDOW* paramsWin = newwin(
        dynamicParams.size() + staticParams.size() + 7, 40, 1, 40);
    drawParamsWin(paramsWin, staticParams, dynamicParams, 0);

    int selected = 0;
    double delta = 0.2;
    while (true) {
        const double freq = 50.0;
        std::this_thread::sleep_for(
            std::chrono::milliseconds((int)(1000/freq)));
        int input = getch();
        if (input == 'q') {
            break;
        } else if (input == KEY_DOWN) {
            if (selected < dynamicParams.size()+staticParams.size()-1) {
                selected++;
                drawParamsWin(paramsWin, staticParams, dynamicParams, selected);
            }
        } else if (input == KEY_UP) {
            if (selected > 0) {
                selected--;
                drawParamsWin(paramsWin, staticParams, dynamicParams, selected);
            }
        } else if (input == ' ') {
            dynamicParams("enabled") = !dynamicParams("enabled");
            drawStatusWin(statusWin, dynamicParams("enabled"), delta);
        } else if (input == '+') {
            delta *= 2.0;
            drawStatusWin(statusWin, dynamicParams("enabled"), delta);
        } else if (input == '-') {
            delta /= 2.0;
            drawStatusWin(statusWin, dynamicParams("enabled"), delta);
        } else if (input == KEY_LEFT) {
            if (selected < dynamicParams.size()) {
                dynamicParams(selected) -= delta;
            } else {
                staticParams(selected-dynamicParams.size()) -= delta;
            }
            drawParamsWin(paramsWin, staticParams, dynamicParams, selected);
        } else if (input == KEY_RIGHT) {
            if (selected < dynamicParams.size()) {
                dynamicParams(selected) += delta;
            } else {
                staticParams(selected-dynamicParams.size()) += delta;
            }
            drawParamsWin(paramsWin, staticParams, dynamicParams, selected);
        } else if (input == '0') {
            if (selected < dynamicParams.size()) {
                dynamicParams(selected) = 0.0;
            } 
            drawParamsWin(paramsWin, staticParams, dynamicParams, selected);
        }
    }
    
    delwin(titleWin);
    delwin(statusWin);
    delwin(paramsWin);
    endwin();

    return 0;
}

