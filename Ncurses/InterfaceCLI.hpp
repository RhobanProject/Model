#ifndef LEPH_INTERFACECLI_HPP
#define LEPH_INTERFACECLI_HPP

#include <vector>
#include <map>
#include <functional>
#include <string>
#include <ncurses.h>
#include <Types/VectorLabel.hpp>

namespace Leph {

/**
 * InterfaceCLI
 *
 * Generic CLI interface based on
 * NCurses and VectorLabel
 */
class InterfaceCLI
{
    public:

        /**
         * Initialization
         */
        InterfaceCLI(const std::string& title = 
            "Ncurses Interface");

        /**
         * Desalocation (exit Ncurses)
         */
        ~InterfaceCLI();

        /**
         * Add a VectorLabel under given section name
         * for write parameters as reference
         * Vector pointer are used. Do no destruct
         * the given vector before the end of InterfaceCLI
         * instance life
         */
        void addParameters(const std::string& sectionName,
            VectorLabel& vector);

        /**
         * Add a VectorLabel under given section name
         * for values real time display as reference
         * Vector pointer are used. Do no destruct
         * the given vector before the end of InterfaceCLI
         * instance life
         */
        void addMonitors(const std::string& sectionName,
            VectorLabel& vector);

        /**
         * Add an user defined input binding with
         * input key (NCurses), a helping message and
         * a callback function
         */
        void addBinding(int key, const std::string& helpMsg,
            std::function<void()> handler);

        /**
         * Add a user defined status string line
         * as string reference
         * String pointer are used. Do no destruct
         * the given string before the end of InterfaceCLI
         * instance life
         */
        void addStatus(std::string& line);

        /**
         * Refresh screen if needed and handle keybord input.
         * Return false if exit is asked. Else return true.
         * Monitors and Status values are not updated if updateAll is
         * false (save up terminal update time).
         */
        bool tick(bool updateAll = true);

    private:

        /**
         * Structure for input event handler
         */
        struct InputBinding {
            int key;
            std::function<void()> handler;
            std::string help;
        };

        /**
         * Ncurses windows pointer to
         * title, status, parameters and
         * monitors windows
         */
        WINDOW* _titleWin;
        WINDOW* _statusWin;
        WINDOW* _paramsWin;
        WINDOW* _monitorsWin;

        /**
         * Interface title
         */
        std::string _title;

        /**
         * VectorLabel pointer container
         * for parameters update
         * associated with section name
         * And sum of parameter
         */
        std::map<std::string,VectorLabel*> _parameters;
        size_t _sumParams;

        /**
         * VectorLabel pointer container
         * for values monitors
         * associated with section name
         * And sum of monitor
         */
        std::map<std::string,VectorLabel*> _monitors;
        size_t _sumMonitors;

        /**
         * Summed index of selected parameter
         */
        size_t _selected;

        /**
         * Parameters incr/decr delta
         */
        double _paramsDelta;

        /**
         * User added input handler
         */
        std::vector<InputBinding> _userBindings;

        /**
         * User defined status
         */
        std::vector<std::string*> _userStatus;

        /**
         * Draw Ncurses title windows
         */
        void drawTitleWin();

        /**
         * Draw Ncurses status windows
         */
        void drawStatusWin();

        /**
         * Draw Ncurses parameters windows
         */
        void drawParamsWin();
        
        /**
         * Draw Ncurses monitors windows
         */
        void drawMonitorsWin();

        /**
         * Return a reference to selected
         * parameters value
         */
        double& getSelected();
};

}

#endif

