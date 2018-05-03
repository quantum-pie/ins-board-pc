#include "eigenaux.h"

#include <iostream>

namespace eaux
{

void debug_vector(const DynamicVector & vec, std::string name)
{
    std::cout << name + ":" << std::endl;
    for(int i = 0; i < vec.size(); ++i)
    {
    	std::cout << vec[i] << ' ';
    }
    std::cout << std::endl;
}

void debug_matrix(const DynamicMatrix & mtx, std::string name)
{
	std::cout << name + ":" << std::endl;
    for(int i = 0; i < mtx.rows(); ++i)
    {
        for(int j = 0; j < mtx.cols(); ++j)
        {
        	std::cout << mtx(i, j) << ' ';
        }
        std::cout << std::endl;
    }
}

}

