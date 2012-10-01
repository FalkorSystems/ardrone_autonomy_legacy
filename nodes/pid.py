#!/usr/bin/env python

class Pid:
    def __init__( self, Kp = 0.0, Ti = 0.0, Td = 0.0, outputLimit = None ):
        self.Kp = Kp
        self.Ti = Ti
        self.Td = Td

        self.prev_error = 0.0
        self.integral = 0.0

        self.outputLimit = outputLimit
        self.setPointMin = self.setPointMax = 0.0


    def get_output( self, pv, dt ):
        if( pv < self.setPointMax and pv > self.setPointMin ):
            error = 0
        else:
            errorFromMax = self.setPointMax - pv
            errorFromMin = self.setPointMin - pv

            if abs( errorFromMax ) > abs( errorFromMin ):
                error = errorFromMin
            else:
                error = errorFromMax

        self.integral += error * dt

        if dt != 0:
            derivative = ( error - self.prev_error ) / dt
        else:
            derivative = 0

        self.prev_error = error

        if self.Ti == 0.0:
            TiInv = 0
        else:
            TiInv = 1/self.Ti

        output = self.Kp * ( error + TiInv * self.integral + self.Td * derivative )
        if self.outputLimit != None:
            limitedOutput = min( max( output, -self.outputLimit ),
                                 self.outputLimit )
            return limitedOutput
        else:
            return output
