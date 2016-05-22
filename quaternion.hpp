#pragma once

  namespace geom
  {

		/**
		 *  \brief Quaternion class.
		 *
		 *  This class encondes a typical quaternion implementation.
		 */
    class Quaternion {
    public:

		/**
		 *  \brief Quaternion contructor. It initializes the quaternion as a real
		 *         number (null imaginary part) of value 1.0
		 */
		Quaternion();

		/**
		 *  \brief Quaternion contructor. It initializes the quaternion according to
		 *         the given parameters
		 *
		 *  \param x, y, z: Vector part (imaginary)
		 *  \param w: Real part
		 */
		Quaternion(double x, double y, double z, double w);

		double x, y, z, w;

    public:

		/**
		 *  \brief Array-like access operator
		 *
		 *  \param index: Index within [0, 3]
		 *  \return The corresponding quaternion term
		 */
		double operator [](unsigned int index) const;

		/**
		 *  \brief Assignment operator
		 *
		 *  \param q: Quaternion which to assign from
		 */
		void operator =(const Quaternion& q);

		/**
		 *  \brief Quaternion multiplication plus assignment operator
		 *
		 *  \param q: Quaternion which to multiply to
		 */
		void operator *=(const Quaternion& q);

		/**
		 *  \brief Quaternion equal-to operator
		 *
		 *  \param q: Quaternion which to compare against
		 */
		bool operator ==(const Quaternion& q) const;

		/**
		 *  \brief Quaternion not-equal-to operator
		 *
		 *  \param q: Quaternion which to compare against
		 */
		bool operator !=(const Quaternion& q) const;

		/**
		 *  \brief Quaternion multiplication operator
		 *
		 *  \param q: Quaternion which to multiply to
		 */
		Quaternion operator * (const Quaternion& q) const;

		/**
		 *  \brief Clears the quaternion to be a real (null imaginary part) of
		 *         value 1.0
		 */
		void clear();

		/**
		 *  \brief Sets the quaternion to be its conjugate
		 */
		void conjugate();

		/**
		 *  \brief Returns the conjugate quaternion
		 *
		 *  \return Conjugate quaternion
		 */
		Quaternion get_conjugate() const;
    };
  }
