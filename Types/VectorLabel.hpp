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
            _eigenVector(size),
            _labelToIndex(),
            _indexToLabel()
        {
            defaultLabels();
        }

        VectorLabel(const LabelList& labels) :
            _eigenVector(labels.size()),
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
         * Merge the two given VectorLabel and concat
         * them
         */
        static inline VectorLabel merge(const VectorLabel& v1, 
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
         * Print values in CSV format
         */
        inline void printToCSV(std::ostream& os = std::cout) const
        {
            for (size_t i=0;i<_indexToLabel.size();i++) {
                os << _eigenVector(i) << " ";
            }
            os << std::endl;
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
                throw std::logic_error("VectorLabel label error");
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
 * Merge operator
 */
inline VectorLabel operator+(const VectorLabel& v1, const VectorLabel& v2)
{
    return VectorLabel::merge(v1, v2);
}

}

#endif

