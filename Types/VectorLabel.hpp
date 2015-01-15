#ifndef LEPH_VECTORLABEL_HPP
#define LEPH_VECTORLABEL_HPP

#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <sstream>
#include "Types/types.h"

namespace Leph {

/**
 * VectorLabel
 *
 * Associate each Eigen vector element
 * with a string label
 */
class VectorLabel
{
    public:

        /**
         * Mapping container typedef
         */
        typedef std::map<std::string, size_t> LabelContainer;
        typedef std::vector<std::string> IndexContainer;
        typedef std::vector<std::string> LabelList;
        typedef std::vector<std::pair<std::string, double>> PairList;

        /**
         * Initialization
         * If label are not given, default labes
         * are used
         */
        VectorLabel() :
            _eigenVector(),
            _labelToIndex(),
            _indexToLabel()
        {
        }
        VectorLabel(size_t size) :
            _eigenVector(Vector::Zero(size)),
            _labelToIndex(),
            _indexToLabel()
        {
            defaultLabels();
        }

        VectorLabel(const LabelList& labels) :
            _eigenVector(Vector::Zero(labels.size())),
            _labelToIndex(),
            _indexToLabel(labels)
        {
            for (size_t i=0;i<_indexToLabel.size();i++) {
                if (_labelToIndex.count(_indexToLabel[i]) != 0) {
                    throw std::logic_error("VectorLabel label error");
                }
                _labelToIndex[_indexToLabel[i]] = i;
            }
        }
        VectorLabel(const Vector& vect) :
            _eigenVector(vect),
            _labelToIndex(),
            _indexToLabel()
        {
            defaultLabels();
        }
        VectorLabel(const LabelList& labels, 
            const Vector& vect) :
            _eigenVector(vect),
            _labelToIndex(),
            _indexToLabel(labels)
        {
            if (labels.size() != (size_t)vect.size()) {
                throw std::logic_error("VectorLabel size invalid");
            }
            for (size_t i=0;i<_indexToLabel.size();i++) {
                if (_labelToIndex.count(_indexToLabel[i]) != 0) {
                    throw std::logic_error("VectorLabel label error");
                }
                _labelToIndex[_indexToLabel[i]] = i;
            }
        }
        VectorLabel(const PairList& pairs) :
            _eigenVector(pairs.size()),
            _labelToIndex(),
            _indexToLabel()
        {
            for (size_t i=0;i<pairs.size();i++) {
                if (_labelToIndex.count(pairs[i].first) != 0) {
                    throw std::logic_error("VectorLabel label error");
                }
                _eigenVector(i) = pairs[i].second;
                _labelToIndex[pairs[i].first] = i;
                _indexToLabel.push_back(pairs[i].first);
            }
        }

        /**
         * Variadic template label, value
         * mapping initialization
         */
        VectorLabel(const std::string& label, double value) :
            _eigenVector(),
            _labelToIndex(),
            _indexToLabel()
        {
            appendAux(label, value);
        }
        template <class ... LabelsValues>
        VectorLabel(const std::string& label, 
            double value, LabelsValues... labelsValues) :
            _eigenVector(),
            _labelToIndex(),
            _indexToLabel()
        {
            appendAux(label, value);
            append(labelsValues...);
        }

        /**
         * Append given label and optional value
         * to the vector container
         */
        inline void append(const std::string& label, double value = 0.0)
        {
            appendAux(label, value);
        }
        template <class ... LabelsValues>
        inline void append(const std::string& label, 
            double value, LabelsValues... labelsValues)
        {
            appendAux(label, value);
            append(labelsValues...);
        }

        /**
         * Direct access to Eigen vector
         */
        inline const Vector& vect() const
        {
            return _eigenVector;
        }
        inline Vector& vect()
        {
            return _eigenVector;
        }

        /**
         * Direct access to labels container
         */
        inline const LabelContainer& labels() const
        {
            return _labelToIndex;
        }
        
        /**
         * Return the vector size
         */
        inline size_t size() const
        {
            return _labelToIndex.size();
        }

        /**
         * Mapping from label to index and back
         */
        inline const std::string& getLabel(size_t index) const
        {
            if (index >= size()) {
                throw std::logic_error("VectorLabel unbound index");
            }

            return _indexToLabel[index];
        }
        inline size_t getIndex(const std::string& label) const
        {
            return _labelToIndex.at(label);
        }

        /**
         * Return true if the given labels
         * is registered
         */
        inline bool exist(const std::string& label) const
        {
            return (_labelToIndex.count(label) > 0);
        }

        /**
         * Access to named vector element
         */
        inline const double& operator()(const std::string& label) const
        {
            size_t index;
            try {
                index = _labelToIndex.at(label);
            } catch (const std::out_of_range& err) {
                throw std::logic_error("VectorLabel invalid label: " + label);
            }

            return _eigenVector(index);
        }
        inline double& operator()(const std::string& label)
        {
            size_t index;
            try {
                index = _labelToIndex.at(label);
            } catch (const std::out_of_range& err) {
                throw std::logic_error("VectorLabel invalid label: " + label);
            }

            return _eigenVector(index);
        }

        /**
         * Access to indexed element
         */
        inline const double& operator()(size_t index) const
        {
            return _eigenVector(index);
        }
        inline double& operator()(size_t index)
        {
            return _eigenVector(index);
        }

        /**
         * Merge the given VectorLabel to this
         * Value priority to given VectorLabel
         * Result labels are the union of given vector and this
         * (All values in given vector are created or update in this)
         */
        inline void mergeUnion(const VectorLabel& v)
        {
            for (const auto& label : v.labels()) {
                if (!exist(label.first)) {
                    append(label.first, v(label.first));
                } else {
                    operator()(label.first) = v(label.first);
                }
            }
        }
        
        /**
         * Merge the given VectorLabel to this
         * Value priority to given VectorLabel
         * Result labels are the intersection of given 
         * vector and this and union of this.
         * (Only values present in this and given vector
         * are updated. Others are not update either deleted)
         */
        inline void mergeInter(const VectorLabel& v)
        {
            for (const auto& label : v.labels()) {
                if (exist(label.first)) {
                    operator()(label.first) = v(label.first);
                } 
            }
        }

        /**
         * Merge the two given VectorLabel and concat
         * them (value priority to first parameter)
         * Result labels are the union of given vectors labels
         */
        static inline VectorLabel mergeUnion(const VectorLabel& v1, 
            const VectorLabel& v2)
        {
            VectorLabel merged = v1;
            for (const auto& label : v2.labels()) {
                if (!merged.exist(label.first)) {
                    merged.append(label.first, v2(label.first));
                } 
            }

            return merged;
        }
        
        /**
         * Merge the two given VectorLabel 
         * (value priority to first parameter)
         * Result labels are the intersection of given vectors labels
         */
        static inline VectorLabel mergeInter(const VectorLabel& v1, 
            const VectorLabel& v2)
        {
            VectorLabel merged;
            for (const auto& label : v1.labels()) {
                if (v2.exist(label.first)) {
                    merged.append(label.first, v1(label.first));
                } 
            }

            return merged;
        }

        /**
         * Print utility
         */
        inline void print(std::ostream& os = std::cout) const
        {
            unsigned int maxLength = 0;
            unsigned int maxDigit = _indexToLabel.size() > 10 ? 2 : 1;
            for (size_t i=0;i<_indexToLabel.size();i++) {
                if (_indexToLabel[i].length() > maxLength) {
                    maxLength = _indexToLabel[i].length();
                }
            }
            for (size_t i=0;i<_indexToLabel.size();i++) {
                os << "[" << std::left << std::setw(maxDigit) << i << ":" 
                   << std::left << std::setw(maxLength) << _indexToLabel[i] << "]" 
                   << " " << _eigenVector(i) << std::endl;
            }
        }

        /**
         * Write values in CSV format to given stream
         */
        inline void writeToCSV(std::ostream& os = std::cout) const
        {
            os << "# ";
            for (size_t i=0;i<_indexToLabel.size();i++) {
                os << _indexToLabel[i] << " ";
            }
            os << std::endl;
            for (size_t i=0;i<_indexToLabel.size();i++) {
                os << std::setprecision(10) << _eigenVector(i) << " ";
            }
            os << std::endl;
        }

        /**
         * Read and load values in CSV format from 
         * given string
         */
        inline void readFromCSV(const std::string& str)
        {
            //Skip empty input
            if (str.length() == 0) return;
            //Check labels line is commented
            if (str[0] != '#') throw std::logic_error(
                "VectorLabel invalid CSV format (first comment line): " + str);
            //Find first label
            size_t index = 0;
            index = str.find_first_not_of(std::string("# "), index);
            if (index == std::string::npos) throw std::logic_error(
                "VectorLabel invalid CSV format (no label): " + str);
            //Init extracted labels and CSV index mapping
            std::map<size_t, std::string> mapping;
            size_t labelIndex = 0;
            //Extract all labels until newline
            while (index != std::string::npos && str[index] != '\n') {
                size_t endLabel = str.find_first_of(std::string(" \n"), index);
                std::string label = str.substr(index, endLabel-index);
                if (!exist(label)) {
                    append(label, 0.0);
                } 
                mapping[labelIndex] = label;
                index = str.find_first_not_of(std::string(" "), endLabel);
                labelIndex++;
            }
            //Go through new line
            index = str.find_first_not_of(std::string("\n\r"), index);
            //Extract all values
            labelIndex = 0;
            while (index != std::string::npos && str[index] != '\n') {
                size_t endLabel = str.find_first_of(std::string(" \n"), index);
                std::string value = str.substr(index, endLabel-index);
                if (mapping.count(labelIndex) == 0) throw std::logic_error(
                    "VectorLabel invalid CSV format (mismatch values labels): " + str);
                operator()(mapping.at(labelIndex)) = std::atof(value.c_str());
                index = str.find_first_not_of(std::string(" "), endLabel);
                labelIndex++;
            }
            //Check labels and values match
            if (labelIndex != mapping.size()) throw std::logic_error(
                "VectorLabel invalid CSV format (mismatch values labels): " + str);
        }
        
        /**
         * Read one vector label data (two lines) from
         * given stream.
         * Return false when given stream is not good anymore
         * else return true.
         */
        inline bool readFromCSV(std::istream& is)
        {
            std::string line1;
            std::string line2;
            getline(is, line1);
            getline(is, line2);
           
            if (line1.length() != 0 || line2.length() != 0) {
                readFromCSV(line1 + "\n" + line2);
            } 
                
            return is.good();
        }

    private:

        /**
         * Double dynamic Eigen values
         * container vector
         */
        Vector _eigenVector;

        /**
         * String label to index mapping
         * container
         */
        LabelContainer _labelToIndex;

        /**
         * Index to string label mapping
         * container
         */
        IndexContainer _indexToLabel;

        /**
         * Build up default label
         */
        inline void defaultLabels()
        {
            for (size_t i=0;i<(size_t)_eigenVector.size();i++) {
                std::ostringstream oss; 
                oss << "label " << i;
                _labelToIndex[oss.str()]  = i;
                _indexToLabel.push_back(oss.str());
            }
        }
        
        /**
         * Append implementation
         */
        inline void appendAux(const std::string& label, double value)
        {
            if (_labelToIndex.count(label) != 0) {
                throw std::logic_error("VectorLabel label already exists");
            }
            size_t len = size();
            _eigenVector.conservativeResize(len+1, Eigen::NoChange_t());
            _eigenVector(len) = value;
            _labelToIndex[label] = len;
            _indexToLabel.push_back(label);
        }
};

/**
 * Overload stream operator
 */
inline std::ostream& operator<<(std::ostream& os, const VectorLabel& vect)
{
    vect.print(os);
    return os;
}

/**
 * Merge union operator
 */
inline VectorLabel operator+(const VectorLabel& v1, const VectorLabel& v2)
{
    return VectorLabel::mergeUnion(v1, v2);
}

}

#endif

