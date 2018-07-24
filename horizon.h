/*
 * horizon.h
 *
 *      Author: bigaw
 */

#ifndef INCLUDE_HORIZON_H_
#define INCLUDE_HORIZON_H_

/*!
 * @brief Orientation horizon adjust class.
 */
class Horizon
{
public:
	/*!
	 * @brief Default constructor.
	 */
	Horizon();

	/*!
	 * @brief Set reference horizon.
	 * @param roll horizon roll.
	 * @param pitch horizon pitch.
	 */
	void set(double roll, double pitch);

	/*!
	 * @brief Reset horizon to default.
	 */
	void reset();

	/*!
	 * @brief Load horizon from file.
	 * @return true of loaded successful.
	 */
	bool load();

	/*!
	 * @brief Save horizon to file.
	 * @return true if saved successful.
	 */
	bool save() const;

	/*!
	 * @brief Get horizon pitch.
	 * @return horizon pitch.
	 */
	double get_pitch() const;

	/*!
	 * @brief Get horizon roll.
	 * @return horizon roll.
	 */
	double get_roll() const;

private:
	double pitch;
	double roll;
};

#endif /* INCLUDE_HORIZON_H_ */
