c     hohmann.for           January 14, 2006

c     two-impulse Hohmann transfer

c     Orbital Mechanics with Fortran

c     *********************************

      use dflib

      implicit double precision (a-h, o-z)

      external hohmfunc

      integer(4) isys

      common /chohmann/ hn1, hn2, hn3, dinc

c     define astrodynamic and utility constants

      pi = 3.14159265358979d0

      pi2 = 2.0d0 * pi

      dtr = pi / 180.0d0

      rtd = 180.0d0 / pi

c     Earth gravitational constant (km**3/sec**2)

      data xmu /398600.5d0/

c     Earth equatorial radius (kilometers)

      data req /6378.14d0/

      isys = system("cls")

      print *, ' '
      print *, 'program hohmann'
      print *, 'two impulse Hohmann transfer'
      print *, '----------------------------'

c     request initial altitude

      do while (.true.)
         print *, '     '
         print *, '     '

         print *, 'please input the initial altitude (kilometers)'

         read (5, *) alt1

         if (alt1 .gt. 0.0d0) exit
      enddo

c     request final altitude

      do while (.true.)
         print *, '     '
         print *, '     '

         print *, 'please input the final altitude (kilometers)'

         read (5, *) alt2

         if (alt2 .gt. alt1) exit
      enddo

c     request initial inclination

      do while (.true.)
         print *, '     '
         print *, '     '

         print *, 'please input the initial inclination (degrees)'

         read (5, *) xinc1

         if (xinc1 .ge. 0.0d0) exit
      enddo

c     request final inclination

      do while (.true.)
         print *, '     '
         print *, '     '

         print *, 'please input the final inclination (degrees)'

         read (5, *) xinc2

         if (xinc2 .ge. 0.0d0) exit
      enddo

c     calculate total inclination change

      dinc = abs(xinc2 * dtr - xinc1 * dtr)

c     compute geocentric radii of initial and final orbits (km)

      r1 = req + alt1

      r2 = req + alt2

c     compute "normalized" radii

      hn1 = sqrt(2.0d0 * r2 / (r2 + r1))

      hn2 = sqrt(r1 / r2)

      hn3 = sqrt(2.0d0 * r1 / (r2 + r1))

c     compute "local circular velocity" of initial and final orbits

      v1 = sqrt(xmu / r1)

      v2 = sqrt(xmu / r2)

c     compute perigee and apogee velocities of the transfer orbit

      vt1 = sqrt(2.0d0 * xmu * r2 / (r1 * (r1 + r2)))

      vt2 = sqrt(2.0d0 * xmu * r1 / (r2 * (r1 + r2)))

c     compute transfer orbit eccentricity

      ecct = (max(r1, r2) - min(r1, r2)) / (r1 + r2)

      if (abs(dinc) .eq. 0.0d0) then
c        coplanar orbit transfer
c        -----------------------

         dv1 = vt1 - v1

         dv2 = v2 - vt2

         dinc1 = 0.0d0

         dinc2 = 0.0d0
      else
c        non-coplanar orbit transfer
c        ---------------------------

         xr1 = 0.0d0

         xr2 = dinc

         rtol = 1.0d-8

         call root (hohmfunc, xr1, xr2, rtol, xroot, froot)

c        calculate delta-v's

         dinc1 = xroot

         dinc2 = dinc - dinc1

         dv1 = v1 * sqrt(1.0d0 + hn1 * hn1 - 2.0d0 * hn1 * cos(dinc1))

         dv2 = v1 * sqrt(hn2 * hn2 * hn3 * hn3 + hn2 * hn2
     &         - 2.0d0 * hn2 * hn2 * hn3 * cos(dinc2))
      end if

c     print results

      isys = system("cls")

      print *, ' '
      print *, 'program hohmann'
      print *, '---------------'
      print *, ' '

      write(*, 110) alt1
  110 format(/, 2x, 'initial altitude     ', 2x, f12.6,
     &               ' kilometers')

      write(*, 120) alt2
  120 format(/, 2x, 'final altitude       ', 2x, f12.6,
     &              ' kilometers')

      write(*, 130) xinc1
  130 format(//, 2x, 'initial inclination  ', 2x, f12.6,
     &              ' degrees')

      write(*, 140) xinc2
  140 format(/, 2x, 'final inclination    ', 2x, f12.6,
     &              ' degrees')

      write(*, 150) 1000.0d0 * dv1
  150 format(//, 2x, 'initial delta-v      ', 2x, f12.6,
     &              ' meters/second')

      write(*, 160) 1000.0d0 * dv2
  160 format(/, 2x, 'final delta-v        ', 2x, f12.6,
     &              ' meters/second')

      write(*, 170) rtd * dinc1
  170 format(//, 2x, 'initial plane change ', 2x, f12.6,
     &              ' degrees')

      write(*, 180) rtd * dinc2
  180 format(/, 2x, 'final plane change   ', 2x, f12.6,
     &              ' degrees')

      end

c     ***************************
c     ***************************

      subroutine hohmfunc (x, fx)

c     Hohmann transfer inclination function

c     ***********************************

      implicit double precision(a-h, o-z)

      common /chohmann/ hn1, hn2, hn3, dinc

      dinc1 = x

      hn = hn2 * hn2

      a = hn1 * sin(dinc1) / sqrt(1.0d0 + hn1 * hn1
     &    - 2.0d0 * hn1 * cos(dinc1))
   
      b = hn * hn3 * (sin(dinc) * cos(dinc1) - cos(dinc) * sin(dinc1))
   
c     calculate objective function value

      fx = a - b / sqrt(hn * hn3 * hn3 + hn
     &     - 2.0d0 * hn * hn3 * cos(dinc - dinc1))

      return
      end

