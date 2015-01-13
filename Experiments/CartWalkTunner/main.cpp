#include <iostream>
#include <ncurses.h>
#include <signal.h>

void resizeHandler(int)
{
    printw("pouet");
    refresh();
}

int main()
{
    //Set handler for terminal resizing
    signal(SIGWINCH, resizeHandler);

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

    move(0, COLS/2-35/2);
    refresh();

    WINDOW* titleWin = newwin(8, 40, 1, 1);
    wborder(titleWin, ACS_VLINE, ACS_VLINE, ACS_HLINE, ACS_HLINE, '+', '+', '+', '+');
    wattrset(titleWin, A_BOLD | A_UNDERLINE);
    mvwprintw(titleWin, 1, 1, "CartWalk Tunner (NCurses)");
    wattrset(titleWin, A_NORMAL);
    mvwprintw(titleWin, 3, 1, "q:      quit");
    mvwprintw(titleWin, 4, 1, "i:      init position");
    mvwprintw(titleWin, 5, 1, "space:  toggle enabled");
    mvwprintw(titleWin, 6, 1, "escape: emergency");
    wrefresh(titleWin);
    
    WINDOW* statusWin = newwin(5, 40, 10, 1);
    wborder(statusWin, ACS_VLINE, ACS_VLINE, ACS_HLINE, ACS_HLINE, '+', '+', '+', '+');
    wattrset(statusWin, A_BOLD);
    mvwprintw(statusWin, 1, 1, "Status");
    wattrset(statusWin, A_NORMAL);
    mvwprintw(statusWin, 3, 1, "Enabled: false");
    wrefresh(statusWin);
    
    WINDOW* paramsWin = newwin(7, 40, 16, 1);
    wborder(paramsWin, '|', '|', '-', '-', '#', '#', '#', '#');
    wattrset(paramsWin, A_BOLD);
    mvwprintw(paramsWin, 1, 1, "Parameters");
    wattrset(paramsWin, A_NORMAL);
    mvwprintw(paramsWin, 3, 1, "# step");
    mvwprintw(paramsWin, 4, 1, "# lateral");
    mvwprintw(paramsWin, 5, 1, "# turn");
    wrefresh(paramsWin);

    while (getch() == ERR);
    
    delwin(titleWin);
    endwin();

    return 0;
}

