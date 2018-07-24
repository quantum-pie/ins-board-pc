/*
 * horizon.cpp
 *
 *      Author: bigaw
 */

#include "horizon.h"

#include <fstream>

#include <boost/lexical_cast.hpp>

Horizon::Horizon()
{
	reset();
	load();
}

bool Horizon::load()
{
    std::ifstream ifile("res/horizon.dat", std::ios::binary);

    if(ifile.is_open())
    {
    	ifile.read(reinterpret_cast<char *>(&roll), sizeof(roll));
    	ifile.read(reinterpret_cast<char *>(&pitch), sizeof(pitch));
        ifile.close();
        return true;
    }
    else
	{
    	return false;
    }
}

bool Horizon::save() const
{
    std::ofstream ofile("res/horizon.dat", std::ios::binary);

    if(ofile.is_open())
    {
    	ofile.write(reinterpret_cast<const char *>(&roll), sizeof(roll));
    	ofile.write(reinterpret_cast<const char *>(&pitch), sizeof(pitch));
    	ofile.close();
    	return true;
    }
    else
	{
		return false;
	}
}

void Horizon::set(double r, double p)
{
	pitch = p;
	roll = r;
}

void Horizon::reset()
{
	pitch = 0;
	roll = 0;
}

double Horizon::get_pitch() const
{
	return pitch;
}

double Horizon::get_roll() const
{
	return roll;
}
