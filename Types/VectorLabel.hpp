#ifndef LEPH_VECTORLABEL_HPP
#define LEPH_VECTORLABEL_HPP

#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <Eigen/Dense>

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
        typedef Eigen::VectorXd EigenContainer;

        /**
         * Initialization
         * If label are not given, default labes
         * are used
         */
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
        VectorLabel(const EigenContainer& vect) :
            _eigenVector(vect),
            _labelToIndex(),
            _indexToLabel()
        {
            defaultLabels();
        }
        VectorLabel(const LabelList& labels, 
            const EigenContainer& vect) :
            _eigenVector(vect),
            _labelToIndex(),
            _indexToLabel(labels)
        {
            if (labels.size() != vect.size()) {
                throw std::logic_error("VectorLabel size invalid");
            }
            for (size_t i=0;i<_indexToLabel.size();i++) {
                if (_labelToIndex.count(_indexToLabel[i]) != 0) {
                    throw std::logic_error("VectorLabel label error");
                }
                _labelToIndex[_indexToLabel[i]] = i;
            }
        }

        /**
         * Assignement operator is forgiden
         */
        VectorLabel& operator=(const VectorLabel&) = delete;

        /**
         * Direct access to Eigen vector
         */
        inline const EigenContainer& vect() const
        {
            return _eigenVector;
        }
        inline EigenContainer& vect()
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
         * Access to named vector element
         */
        inline const double& operator()(const std::string& label) const
        {
            return _eigenVector(_labelToIndex.at(label));
        }
        inline double& operator()(const std::string& label)
        {
            return _eigenVector(_labelToIndex.at(label));
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
         * Print utility
         */
        inline void print(std::ostream& os = std::cout) const
        {
            for (size_t i=0;i<_indexToLabel.size();i++) {
                os << "[" << i << ":" << _indexToLabel[i] << "] ";
                os << _eigenVector(i) << std::endl;
            }
        }

    private:

        /**
         * Double dynamic Eigen values
         * container vector
         */
        EigenContainer _eigenVector;

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
        void defaultLabels()
        {
            for (size_t i=0;i<_eigenVector.size();i++) {
                std::ostringstream oss; 
                oss << "label " << i;
                _labelToIndex[oss.str()]  = i;
                _indexToLabel.push_back(oss.str());
            }
        }
};

/**
 * Overload stream operator
 */
std::ostream& operator<<(std::ostream& os, const VectorLabel& vect)
{
    vect.print(os);
    return os;
}

}

#endif

