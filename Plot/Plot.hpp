#ifndef LEPH_PLOT_HPP
#define LEPH_PLOT_HPP

#include <vector>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>
#include "Utils/time.h"
#include "Types/VectorLabel.hpp"
#include "TimeSeries/TimeSeries.hpp"

namespace Leph {

/**
 * Plot
 *
 * Programming interface with
 * the Linux ploting utility 
 * Gnuplot using VectorLabel
 */
class Plot
{
    public:

        /**
         * Plot style enumeration
         */
        enum Style {
            Points,
            Lines,
            LinesPoints,
            ErrorsPoints,
            ErrorsLines,
            None,
        };

        /**
         * Initialization
         */
        Plot() :
            _rangeMinX(1.0),
            _rangeMaxX(0.0),
            _rangeMinY(1.0),
            _rangeMaxY(0.0),
            _rangeMinZ(1.0),
            _rangeMaxZ(0.0),
            _pipeFd(-1)
        {
        }

        /**
         * Add a (vector) point
         */
        inline void add(const VectorLabel& vect)
        {
            _database.push_back(vect);
        }

        /**
         * Add given TimeSeries with optionally
         * an other name
         */
        inline void add(const TimeSeries& series)
        {
            for (size_t i=0;i<series.size();i++) {
                _database.push_back(VectorLabel(
                    "time", series[i].time,
                    series.name(), series[i].value));
            }
        }
        inline void add(const std::string& name, 
            const TimeSeries& series)
        {
            for (size_t i=0;i<series.size();i++) {
                _database.push_back(VectorLabel(
                    "time", series[i].time,
                    name, series[i].value));
            }
        }

        /**
         * Request a 2D plot with X and Y axis
         * X axis could be "index"
         * Y axis could be "all"
         * Y axis could be "filter:*"
         */
        inline Plot& plot(const std::string& xAxis, 
            const std::string& yAxis, 
            const Style style = LinesPoints,
            const std::string& palette = "")
        {
            for (size_t i=0;i<_plots2D.size();i++) {
                if (_plots2D[i].xAxis == xAxis && _plots2D[i].yAxis == yAxis) {
                    _plots2D[i].xAxis = xAxis;
                    _plots2D[i].yAxis = yAxis;
                    _plots2D[i].style = style;
                    _plots2D[i].palette = palette;
                    return *this;
                }
            }

            if (yAxis == "all") {
                for (size_t i=0;i<_database.size();i++) {
                    for (const auto& label : _database[i].labels()) {
                        if (label.first != xAxis) {
                            plot(xAxis, label.first, style, palette);
                        }
                    }
                }
                return *this;
            } else if (yAxis.size() > 2 && 
                yAxis[yAxis.size()-1] == '*' && 
                yAxis[yAxis.size()-2] == ':'
            ) {
                std::string filter = yAxis.substr(0, 
                    yAxis.find_first_of(':'));
                for (size_t i=0;i<_database.size();i++) {
                    for (const auto& label : _database[i].labels()) {
                        if (label.first != xAxis && 
                            VectorLabel::toSection(label.first) == filter
                        ) {
                            plot(xAxis, label.first, style, palette);
                        }
                    }
                }
                return *this;
            } else {
                if (
                    (style == ErrorsLines || 
                    style == ErrorsPoints) &&
                    palette == ""
                ) {
                    throw std::logic_error(
                        "Plot errors need third column");
                }
                Plot2D request;
                request.xAxis = xAxis;
                request.yAxis = yAxis;
                request.style = style;
                request.palette = palette;
                _plots2D.push_back(request);
                return *this;
            }
        }
        
        /**
         * Request a 3D plot with X, Y and Z axis 
         * X axis could be "index"
         * Y axis could be "index"
         * Z axis could be "all"
         * Z axis could be "filter:*"
         * Z axis could be "ZERO"
         */
        inline Plot& plot(const std::string& xAxis, 
            const std::string& yAxis, const std::string& zAxis,
            const Style style = Points,
            const std::string& palette = "")
        {
            for (size_t i=0;i<_plots3D.size();i++) {
                if (_plots3D[i].xAxis == xAxis && 
                    _plots3D[i].yAxis == yAxis &&
                    _plots3D[i].zAxis == zAxis
                ) {
                    _plots3D[i].xAxis = xAxis;
                    _plots3D[i].yAxis = yAxis;
                    _plots3D[i].zAxis = zAxis;
                    _plots3D[i].style = style;
                    _plots3D[i].palette = palette;
                    return *this;
                }
            }

            if (zAxis == "all") {
                for (size_t i=0;i<_database.size();i++) {
                    for (const auto& label : _database[i].labels()) {
                        if (label.first != xAxis && label.first != yAxis) {
                            plot(xAxis, yAxis, label.first, style, palette);
                        }
                    }
                }
                return *this;
            } else if (zAxis.size() > 2 && 
                zAxis[zAxis.size()-1] == '*' && 
                zAxis[zAxis.size()-2] == ':'
            ) {
                std::string filter = zAxis.substr(0, 
                    zAxis.find_first_of(':'));
                for (size_t i=0;i<_database.size();i++) {
                    for (const auto& label : _database[i].labels()) {
                        if (label.first != xAxis && 
                            VectorLabel::toSection(label.first) == filter
                        ) {
                            plot(xAxis, yAxis, label.first, style, palette);
                        }
                    }
                }
                return *this;
            } else {
                if (
                    style == ErrorsLines || 
                    style == ErrorsPoints
                ) {
                    throw std::logic_error(
                        "Plot errors not implemented in 3D");
                }
                Plot3D request;
                request.xAxis = xAxis;
                request.yAxis = yAxis;
                request.zAxis = zAxis;
                request.style = style;
                request.palette = palette;
                _plots3D.push_back(request);
                return *this;
            }
        }

        /**
         * Set x,y,z ploting range
         */
        inline Plot& rangeX(double min, double max)
        {
            _rangeMinX = min;
            _rangeMaxX = max;

            return *this;
        }
        inline Plot& rangeY(double min, double max)
        {
            _rangeMinY = min;
            _rangeMaxY = max;

            return *this;
        }
        inline Plot& rangeZ(double min, double max)
        {
            _rangeMinZ = min;
            _rangeMaxZ = max;

            return *this;
        }

        /**
         * Render the plot
         * Wait until plot window is closed
         */
        inline void render(const std::string& exportFile = "")
        {
            if (_plots2D.size() > 0 && _plots3D.size() > 0) {
                throw std::logic_error("Plot error 2d and 3d request");
            }
            if (_plots2D.size() == 0 && _plots3D.size() == 0) {
                return;
            }

            std::string commands = generatePlotting();
            
            //Dump gnuplot script to file
            std::ofstream file;
            if (exportFile == "") {
                file.open("/tmp/plot-" + Leph::currentDate() + ".plot");
            } else {
                file.open(exportFile);
            }
            file << commands;
            file.close();

            //Send to gnuplot instance
            if (exportFile == "") {
                createGnuplotInstance();
                if (_pipeFd <= 0) {
                    throw std::logic_error("Plot closed pipe fd");
                }
                int n = write(_pipeFd, commands.c_str(), commands.length());
                if (n == -1) {
                  throw std::runtime_error("failed to write to gnuplot");
                }
                waitCloseGnuplotInstance();
            }
            
            clearPlots();
        }

        /**
         * Reset points database, plot
         * requests and both
         */
        inline void clearPoints()
        {
            _database.clear();
        }
        inline void clearPlots()
        {
            _plots2D.clear();
            _plots3D.clear();
        }
        inline void clear()
        {
            clearPoints();
            clearPlots();
        }

    private:

        /**
         * Plot request 2D and 3D
         * structures
         */
        struct Plot2D {
            std::string xAxis;
            std::string yAxis;
            Style style;
            std::string palette;
        };
        struct Plot3D {
            std::string xAxis;
            std::string yAxis;
            std::string zAxis;
            Style style;
            std::string palette;
        };

        /**
         * Vector labeled points database
         */
        std::vector<VectorLabel> _database;

        /**
         * Plot request container
         */
        std::vector<Plot2D> _plots2D;
        std::vector<Plot3D> _plots3D;

        /**
         * Plot x,y,z min,max range
         * If max > min, auto scaling is used
         */
        double _rangeMinX;
        double _rangeMaxX;
        double _rangeMinY;
        double _rangeMaxY;
        double _rangeMinZ;
        double _rangeMaxZ;
        
        /**
         * Gnuplot pipe file descriptor
         */
        int _pipeFd;
        
        /**
         * Fork current process to create a new GnuPlot window
         */
        inline void createGnuplotInstance()
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
                _pipeFd = pipefd[1];
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
                execlp("gnuplot", "gnuplot", "-", NULL);
            } else {
                throw std::runtime_error("Plot failed to fork");
            }
        }

        /**
         * Wait for end of gnuplot session and
         * close the openned pipe to GnuPlot window instance
         */
        inline void waitCloseGnuplotInstance()
        {
            waitpid(-1, NULL, 0);
            if (_pipeFd != -1) {
                close(_pipeFd);
                _pipeFd = -1;
            }
        }

        /**
         * Generate and return Gnuplot commands and data
         */
        inline std::string generatePlotting()
        {
            std::string commands;
            std::string data;

            commands += "set grid;\n";
            if (_rangeMinX < _rangeMaxX) {
                std::ostringstream oss;
                oss << "set xrange[" << _rangeMinX;
                oss << ":" << _rangeMaxX << "];\n";
                commands += oss.str();
            }
            if (_rangeMinY < _rangeMaxY) {
                std::ostringstream oss;
                oss << "set yrange[" << _rangeMinY;
                oss << ":" << _rangeMaxY << "];\n";
                commands += oss.str();
            }
            if (_rangeMinZ < _rangeMaxZ) {
                std::ostringstream oss;
                oss << "set zrange[" << _rangeMinZ;
                oss << ":" << _rangeMaxZ << "];\n";
                commands += oss.str();
            }

            //Create commands and data to send to gnuplot
            if (_plots2D.size() != 0) {
                commands += "plot ";
            } else {
                commands += "splot ";
            }
            
            bool isFirst = true;
            for (size_t i=0;i<_plots2D.size();i++) {
                if (_plots2D[i].style == None) {
                    continue;
                }
                bool isPalette = _plots2D[i].palette != "";
                if (!isFirst) {
                    commands += ", ";
                }
                isFirst = false;
                if (isPalette) {
                    if (
                        _plots2D[i].style == ErrorsLines || 
                        _plots2D[i].style == ErrorsPoints
                    ) {
                        commands += "'-' using 1:2:3 with ";
                    } else {
                        commands += "'-' using 1:2:3 palette with ";
                    }
                } else {
                    commands += "'-' using 1:2 with ";
                }
                if (_plots2D[i].style == Points) {
                    commands += "points";
                }
                if (_plots2D[i].style == Lines) {
                    commands += "lines";
                }
                if (_plots2D[i].style == LinesPoints) {
                    commands += "linespoints";
                }
                if (_plots2D[i].style == ErrorsPoints) {
                    commands += "yerrorpoints";
                }
                if (_plots2D[i].style == ErrorsLines) {
                    commands += "yerrorlines";
                }
                if (isPalette) {
                    commands += " title '" + _plots2D[i].xAxis 
                        + " --> " + _plots2D[i].yAxis + " // " + _plots2D[i].palette + "' ";
                } else {
                    commands += " title '" + _plots2D[i].xAxis 
                        + " --> " + _plots2D[i].yAxis + "' ";
                }
                for (size_t j=0;j<_database.size();j++) {
                    if (
                        (_plots2D[i].xAxis != "index" &&
                        !_database[j].exist(_plots2D[i].xAxis)) || 
                        !_database[j].exist(_plots2D[i].yAxis) ||
                        (isPalette && 
                        _plots2D[i].palette != "index" &&
                        !_database[j].exist(_plots2D[i].palette))
                    ) {
                        continue;
                    }
                    std::ostringstream oss;
                    if (_plots2D[i].xAxis == "index") {
                        oss << j << " " << _database[j](_plots2D[i].yAxis);
                    } else {
                        oss << _database[j](_plots2D[i].xAxis) << " " 
                            << _database[j](_plots2D[i].yAxis);
                    }
                    if (isPalette) {
                        if (_plots2D[i].palette == "index") {
                            oss << " " << j;
                        } else {
                            oss << " " << _database[j](_plots2D[i].palette);
                        }
                    }
                    data += oss.str() + "\n";
                }
                data += "end\n";
            }
            for (size_t i=0;i<_plots3D.size();i++) {
                if (_plots3D[i].style == None) {
                    continue;
                }
                bool isPalette = _plots3D[i].palette != "";
                if (!isFirst) {
                    commands += ", ";
                }
                isFirst = false;
                if (isPalette) {
                    commands += "'-' using 1:2:3:4 palette with ";
                } else {
                    commands += "'-' using 1:2:3 with ";
                }
                if (_plots3D[i].style == Points) {
                    commands += "points";
                }
                if (_plots3D[i].style == Lines) {
                    commands += "lines";
                }
                if (_plots3D[i].style == LinesPoints) {
                    commands += "linespoints";
                }
                if (isPalette) {
                    commands += " title '" + _plots3D[i].xAxis + "," + _plots3D[i].yAxis 
                        + " --> " + _plots3D[i].zAxis + " // " + _plots3D[i].palette + "' ";
                } else {
                    commands += " title '" + _plots3D[i].xAxis + "," + _plots3D[i].yAxis 
                        + " --> " + _plots3D[i].zAxis + "' ";
                }
                for (size_t j=0;j<_database.size();j++) {
                    if (
                        (_plots3D[i].xAxis != "index" &&
                        !_database[j].exist(_plots3D[i].xAxis)) || 
                        (_plots3D[i].yAxis != "index" &&
                        !_database[j].exist(_plots3D[i].yAxis)) || 
                        (_plots3D[i].zAxis != "ZERO" &&
                        !_database[j].exist(_plots3D[i].zAxis)) ||
                        (isPalette && 
                        _plots3D[i].palette != "index" &&
                        !_database[j].exist(_plots3D[i].palette))
                    ) {
                        continue;
                    }
                    std::ostringstream oss;
                    if (_plots3D[i].xAxis == "index") {
                        oss << j << " ";
                    } else {
                        oss << _database[j](_plots3D[i].xAxis) << " ";
                    }
                    if (_plots3D[i].yAxis == "index") {
                        oss << j << " ";
                    } else {
                        oss << _database[j](_plots3D[i].yAxis) << " ";
                    }
                    if (_plots3D[i].zAxis == "ZERO") {
                        oss << "0.0";
                    } else {
                        oss << _database[j](_plots3D[i].zAxis);
                    }
                    if (isPalette) {
                        if (_plots3D[i].palette == "index") {
                            oss << " " << j;
                        } else {
                            oss << " " << _database[j](_plots3D[i].palette);
                        }
                    }
                    data += oss.str() + "\n";
                }
                data += "end\n";
            }
            commands += ";\n";
            data += "pause mouse close;\nquit;\nquit;\n";

            return commands + data;
        }
};

}

#endif

