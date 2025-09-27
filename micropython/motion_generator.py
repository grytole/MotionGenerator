import time
from math import sqrt

class MotionGenerator:
    def __init__(self, maxVel, maxAcc, initPos):
        self.maxVel = maxVel
        self.maxAcc = maxAcc
        self.initPos = initPos
        self.oldTime = time.ticks_us()
        self.lastTime = self.oldTime
        self.deltaTime = 0
        self.reset()
        self.signOfMotion = 1
        self.is_trapezoid = True
        self.is_finished = False

    def update(self, posRef):
        if self.oldPosRef != posRef:
            self.is_finished = False

            # shift state variables
            self.oldPosRef = posRef
            self.oldPos = self.pos
            self.oldVel = self.vel
            self.oldTime = self.lastTime

            # calc breaking time and distance
            self.tBrk = abs(self.oldVel) / self.maxAcc
            self.dBrk = self.tBrk * abs(self.oldVel) / 2.0

            # calc sign of motion
            oldVelSign = self._sign(self.oldVel)
            oldPosTarget = self.oldPos + oldVelSign * self.dBrk
            self.signOfMotion = self._sign(posRef - oldPosTarget)

            if self.signOfMotion != oldVelSign:
                # brake is needed
                self.tAcc = self.maxVel / self.maxAcc
                self.dAcc = self.tAcc * self.maxVel / 2.0
            else:
                self.tBrk = 0.0
                self.dBrk = 0.0
                self.tAcc = (self.maxVel - abs(self.oldVel)) / self.maxAcc
                self.dAcc = self.tAcc * (self.maxVel + abs(self.oldVel)) / 2.0

            # calc total distance to go after braking
            self.dTot = abs(posRef - self.oldPos + self.signOfMotion * self.dBrk)

            self.tDec = self.maxVel / self.maxAcc
            self.dDec = self.tDec * self.maxVel / 2.0
            self.dVel = self.dTot - (self.dAcc + self.dDec)
            self.tVel = self.dVel / self.maxVel

            if self.tVel > 0:
                self.is_trapezoid = True
            else:
                # not enough time to reach max velocity
                self.is_trapezoid = False

                # recalc distances and periods
                if self.signOfMotion != oldVelSign:
                    # brake is needed
                    self.velSt = sqrt(self.maxAcc * self.dTot)
                    self.tAcc = self.velSt / self.maxAcc
                    self.dAcc = self.tAcc * self.velSt / 2.0
                else:
                    self.tBrk = 0.0
                    self.dBrk = 0.0
                    self.dTot = abs(posRef - self.oldPos)
                    self.velSt = sqrt(0.5 * (self.oldVel**2) + self.maxAcc * self.dTot)
                    self.tAcc = (self.velSt - abs(self.oldVel)) / self.maxAcc
                    self.dAcc = self.tAcc * (self.velSt + abs(self.oldVel)) / 2.0

                self.tDec = self.velSt / self.maxAcc
                self.dDec = self.tDec * self.velSt / 2.0

        currTime = time.ticks_us()
        self.deltaTime = time.ticks_diff(currTime, self.oldTime)

        # calc new setpoint
        if self.is_trapezoid == True:
            self._calculate_trapezoidal_profile(posRef)
        else:
            self._calculate_triangular_profile(posRef)

        self.lastTime = currTime
        return self.pos

    def velocity(self):
        return self.vel

    def acceleration(self):
        return self.acc

    def maxVelocity(self, maxVel=None):
        if maxVel == None:
            return self.maxVel
        else:
            self.maxVel = maxVel

    def maxAcceleration(self, maxAcc=None):
        if maxAcc == None:
            return self.maxAcc
        else:
            self.maxAcc = maxAcc

    def initPosition(self, initPos=None):
        if initPos == None:
            return self.initPos
        else:
            self.initPos = initPos
            self.pos = initPos
            self.oldPos = initPos

    def finished(self):
        return self.is_finished

    def reset(self):
        self.pos = self.initPos
        self.oldPos = self.initPos
        self.oldPosRef = 0.0
        self.vel = 0.0
        self.acc = 0.0
        self.oldVel = 0.0
        self.dBrk = 0.0
        self.dAcc = 0.0
        self.dVel = 0.0
        self.dDec = 0.0
        self.dTot = 0.0
        self.tBrk = 0.0
        self.tAcc = 0.0
        self.tVel = 0.0
        self.tDec = 0.0
        self.velSt = 0.0

    def _sign(self, val):
        if val < 0.0:
            return -1
        elif val > 0.0:
            return 1
        else:
            return 0

    def _calculate_trapezoidal_profile(self, posRef):
        t = self.deltaTime / 1_000_000
        signM = self.signOfMotion
        accelPeriodEnd = self.tBrk + self.tAcc
        coastPeriodEnd = accelPeriodEnd + self.tVel
        decelPeriodEnd = coastPeriodEnd + self.tDec

        if t <= accelPeriodEnd:
            # acceleration phase
            self.acc = signM * self.maxAcc
            currVel = self.acc * t
            self.vel = self.oldVel + currVel
            currPos = 0.5 * currVel * t
            self.pos = self.oldPos + self.oldVel * t + currPos
        elif t < coastPeriodEnd:
            # coasting phase
            self.acc = 0.0
            self.vel = signM * self.maxVel
            dCoastLeft = self.maxVel * (t - self.tBrk - self.tAcc)
            self.pos = self.oldPos + signM * (-self.dBrk + self.dAcc + dCoastLeft)
        elif t < decelPeriodEnd:
            # deceleration phase
            self.acc = -(signM * self.maxAcc)
            tDecLeft = t - self.tBrk - self.tAcc - self.tVel
            currVel = self.maxAcc * tDecLeft
            self.vel = signM * (self.maxVel - currVel)
            dDecLeft = self.maxVel * tDecLeft - 0.5 * currVel * tDecLeft
            self.pos = self.oldPos + signM * (-self.dBrk + self.dAcc + self.dVel + dDecLeft)
        else:
            # target reached
            self.pos = posRef
            self.vel = 0.0
            self.acc = 0.0
            self.is_finished = True

    def _calculate_triangular_profile(self, posRef):
        t = self.deltaTime / 1_000_000
        signM = self.signOfMotion
        accelPeriodEnd = self.tBrk + self.tAcc
        decelPeriodEnd = accelPeriodEnd + self.tDec

        if t <= accelPeriodEnd:
            # acceleration phase
            self.acc = signM * self.maxAcc
            currVel = self.acc * t
            self.vel = self.oldVel + currVel
            currPos = 0.5 * currVel * t
            self.pos = self.oldPos + self.oldVel * t + currPos
        elif t < decelPeriodEnd:
            # deceleration phase
            self.acc = -(signM * self.maxAcc)
            tDecLeft = t - self.tBrk - self.tAcc
            currVel = self.maxAcc * tDecLeft
            self.vel = signM * (self.velSt - currVel)
            dDecLeft = self.velSt * tDecLeft - 0.5 * currVel * tDecLeft
            self.pos = self.oldPos + signM * (-self.dBrk + self.dAcc + dDecLeft)
        else:
            # target reached
            self.pos = posRef
            self.vel = 0.0
            self.acc = 0.0
            self.is_finished = True

