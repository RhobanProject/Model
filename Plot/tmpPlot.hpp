#ifndef LEPH_PLOT_PLOT_HPP
#define LEPH_PLOT_PLOT_HPP

#include <map>
#include <string>
#include <list>
#include <stdexcept>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "Types/VertorLabel.hpp"

namespace Leph {

/**
 * Plot
 *
 * Simple programming interface with
 * ploting utility GnuPlot
 * Uses Vector Label values container
 */
class Plot
{
    public:

        /**
         * Useful Typedef
         */
        typedef std::map<std::string,std::string> OptionContainer;
        typedef std::map<std::string,std::list<PlotPoint2d> > Plot2dContainer;
        typedef std::map<std::string,std::list<PlotPoint3d> > Plot3dContainer;

        /**
         * Clear all internal saved dataset
         */
        static void clear()
        {
            getPlot2d().clear();
            getPlot3d().clear();
        }

        /**
         * Reset all ploting configurations and options
         */
        static void reset()
        {
            getGlobalOptions() = std::string();
            getPlotOptions().clear();
        }

        /**
         * Set GnuPlot global options
         */
        static void option(const std::string& opt)
        {
            getGlobalOptions() = opt;
        }

        /**
         * Set GnuPlot options associated with 
         * specific plot dataset with and without
         * name sufixe
         */
        static void option(const std::string& name, const std::string& opt)
        {
            getPlotOptions()[name] = opt;
        }
        static void option(int index, const std::string& name, const std::string& opt)
        {
            option(suffixe(name, index), opt);
        }

        /**
         * Add a 2d or 3d data point to the given plot dataset 
         * with given name
         */
        static void add(const std::string& name, double y)
        {
            double x;
            if (getPlot2d()[name].size() == 0) {
                x = 0;
            } else {
                x = getPlot2d()[name].back().x + 1.0;
            }
            add(name, x, y);
        }
        static void add(const std::string& name, double x, double y)
        {
            getPlot2d()[name].push_back(PlotPoint2d(x, y));
        }
        static void add(const std::string& name, double x, double y, double z)
        {
            getPlot3d()[name].push_back(PlotPoint3d(x, y, z));
        }
        /**
         * Add a 2d or 3d data point to the given plot dataset
         * with given name sufixed by given integer index
         */
        static void add(int index, const std::string& name, double y)
        {
            add(suffixe(name, index), y);
        }
        static void add(int index, const std::string& name, double x, double y)
        {
            add(suffixe(name, index), x, y);
        }
        static void add(int index, const std::string& name, double x, double y, double z)
        {
            add(suffixe(name, index), x, y, z);
        }

        /**
         * Display all saved datasets to a new GnuPlot window
         */
        static inline void plot()
        {
            if (getPlot2d().size() == 0 && getPlot3d().size() == 0) {
                return;
            }
            closeGnuplotInstance();
            createGnuplotInstance();
            doPlotting();
            closeGnuplotInstance();
            clear();
        }

    private:

        /**
         * Internal access to static global 
         * data structures (private attributes)
         */
        static inline int& getPipeFd()
        {
            //Pipe file descriptor to GnuPLot process
            static int pipeFd = -1;
            return pipeFd;
        }
        static inline std::string& getGlobalOptions()
        {
            //Global GnuPlot options
            static std::string globalOptions;
            return globalOptions;
        }
        static inline OptionContainer& getPlotOptions()
        {
            //Options associated with single dataset
            static OptionContainer plotOptions;
            return plotOptions;
        }
        static inline Plot2dContainer& getPlot2d()
        {
            //Plot 2d datasets container associated with
            //their name
            static Plot2dContainer plot2d;
            return plot2d;
        }
        static inline Plot3dContainer& getPlot3d()
        {
            //Plot 3d datasets container associated with
            //their name
            static Plot3dContainer plot3d;
            return plot3d;
        }

        /**
         * Fork current process to create a new GnuPlot window
         */
        static void createGnuplotInstance()
        {
            //Creating communication pipe
            int pipefd[2];
            if (pipe(pipefd) == -1) {
                throw std::runtime_error("Plot failed to create pipe");
            }
            
            //Forkink current process
            pid_t pid = fork();
            if (pid > 0) {
                //Closing reading pipe end
                close(pipefd[0]);
                //Saving pipe fd
                getPipeFd() = pipefd[1];
            } else if (pid == 0) {
                //Closing writting pipe end
                close(pipefd[1]);
                //Redirecting reading pipe end to standart input
                if (dup2(pipefd[0], STDIN_FILENO) == -1) {
                    throw std::runtime_error("Plot failed to dup2");
                }
                //Closing output and err
                int null = open("/dev/null", O_WRONLY);
                if (dup2(null, STDOUT_FILENO) == -1) {
                    throw std::runtime_error("Plot failed to dup2");
                }
                if (dup2(null, STDERR_FILENO) == -1) {
                    throw std::runtime_error("Plot failed to dup2");
                }
                //Calling Gnuplot
                execlp("gnuplot", "gnuplot", "-p", "-", NULL);
            } else {
                throw std::runtime_error("Plot failed to fork");
            }
        }

        /**
         * Close the openned pipe to GnuPlot window instance
         * (Gnuplot window is not close)
         */
        static void closeGnuplotInstance()
        {
            if (getPipeFd() != -1) {
                close(getPipeFd());
                getPipeFd() = -1;
            }
        }

        /**
         * Send data and plot commands to GnuPlot instance
         */
        static void doPlotting()
        {
            if (getPipeFd() <= 0) {
                throw std::logic_error("Plot closed pipe fd");
            }
            
            std::string commands;
            std::string data;

            //Global options
            if (getGlobalOptions().length() > 0) {
               commands += getGlobalOptions();
               commands += ";";
            }

            //Create commands and data to send to gnuplot
            if (getPlot3d().size() != 0) {
                commands += "splot ";
            } else {
                commands += "plot ";
            }

            Plot3dContainer::const_iterator it1;
            bool isFirst = true;
            for (it1=getPlot3d().begin();it1!=getPlot3d().end();it1++) {
                if (it1->second.size() == 0) {
                    continue;
                }
                if (!isFirst) {
                    commands += ", ";
                }
                isFirst = false;
                commands += "'-' using 1:2:3";
                commands += " title '" + it1->first + "' ";
                if (getPlotOptions().count(it1->first) > 0) {
                    commands += getPlotOptions()[it1->first];
                }
                std::list<PlotPoint3d>::const_iterator it;
                for (it=it1->second.begin();it!=it1->second.end();it++) {
                    std::ostringstream oss;
                    oss << it->x << " " << it->y << " " << it->z;
                    data += oss.str() + "\n";
                }
                data += "end\n";
            }
            Plot2dContainer::const_iterator it2;
            for (it2=getPlot2d().begin();it2!=getPlot2d().end();it2++) {
                if (it2->second.size() == 0) {
                    continue;
                }
                if (!isFirst) {
                    commands += ", ";
                }
                isFirst = false;
                if (getPlot3d().size() != 0) {
                    commands += "'-' using 1:2:3";
                } else {
                    commands += "'-' using 1:2";
                }
                commands += " title '" + it2->first + "' ";
                if (getPlotOptions().count(it2->first) > 0) {
                    commands += getPlotOptions()[it2->first];
                }
                std::list<PlotPoint2d>::const_iterator it;
                for (it=it2->second.begin();it!=it2->second.end();it++) {
                    std::ostringstream oss;
                    if (getPlot3d().size() != 0) {
                        oss << it->x << " " << it->y << " " << 0.0;
                    } else {
                        oss << it->x << " " << it->y;
                    }
                    data += oss.str() + "\n";
                }
                data += "end\n";
            }
            commands += ";\n";

            //Sending commands and data to GnuPlot
            write(getPipeFd(), commands.c_str(), commands.length());
            write(getPipeFd(), data.c_str(), data.length());
        }

        static inline std::string suffixe(const std::string& name, int index)
        {
            std::ostringstream oss;
            oss << name << "_" << index;
            return oss.str();
        }
};

}

#endif

