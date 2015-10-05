#!/usr/bin/python

""" Simulation of digital controllers.

Simulate digital PID controllers on a PT1 system
and draw behavior in a plot

TODO: implement S-IMC controller tuning
TODO: make GUI for configuraiton of controller and control system
"""

import numpy as np
import matplotlib.pyplot as plt

def calcPT2(u, T, d):
    """implementation of a PT2 control system."""   
    
    if not hasattr(calcPT2, "y_old"):
        calcPT2.y_old = 0    
               
    if not hasattr(calcPT2, "y_old_old"):
        calcPT2.y_old_old = 0     
             
    y = (u + calcPT2.y_old * (2*d*T + 2*(T**2)) - calcPT2.y_old_old * (T**2)) / (1 + 2*d*T + T**2)
    
    calcPT2.y_old_old =  calcPT2.y_old
    calcPT2.y_old = y   
     
    return y

def ErrorSignal(t):     
    return 0.0 * np.sin( 2 * np.pi / 50 * t)


def Impulsantwort():
    """calculate the impulse response of the control system."""
    
    T = 100.0
    g = []
    
    for t in range(0,1000):
        err = 1 * np.sin( 2 * np.pi / 50 * t )
        g.append(1/T * np.exp(-t/T) + err)  # 1/T * e^(-t/T) + 
        
    return g


def Strecke(u):  
    """simulation of the control loop."""  
    
    if not hasattr(Strecke, "uList"):
        Strecke.uList = []   
        
    Strecke.uList.append(u)
    
    gList = Impulsantwort()   
    y = 0
    
    i = len(Strecke.uList)-1
    for j in range(0,len(gList)):
        y += gList[j] * Strecke.uList[i]
        i -= 1
        if i < 0 : break    
        
    return y


class Controller(object): 
    """definition of the PID-controller."""  

    def __init__(self, Kp, Ki, Kd, T=1): 
        """c'tor"""
        self._Kp = Kp 
        self._Ki = Ki 
        self._Kd = Kd 
        self._T = T
        self._eAlt = 0
        self._I = 0
    
    def calc(self, e): 
        self._P = self._Kp * e
        self._I += self._Ki * (e + self._eAlt) / 2 * self._T
        self._D = self._Kd * (e - self._eAlt) / self._T
        self._eAlt = e
        return self._P + self._I + self._D


def Sollwert(t):
    """definition of the setpoint-signal."""  
    
    if t < 75:
        ret = t/75.0*20.0
    elif t >= 75 and t < 120:
        ret = 20
    else:
        ret = 10
    return ret


def main():
    """definition of the main function."""  

    SollWerte = []
    StellWerte = []
    IstWerte = []
    
    Iterations = 700
    y = 0    # Istwert
    
    pid = Controller(30.0, 0.7, 100.0)
    
    for t in range(0,Iterations):    
        w = Sollwert(t)
        e = w - y 
        u = pid.calc(e)
        y = calcPT2(u, 50.0, 0.3) + ErrorSignal(t)
        SollWerte.append(w)
        StellWerte.append(u)  
        IstWerte.append(y)
    
    # Create a plot and display it
    
    fig = plt.figure()
    
    # Set common labels
    fig.text(0.5, 0.04, 'time', ha='center', va='center')
    fig.text(0.06, 0.5, 'amplitude', ha='center', va='center', rotation='vertical')
    
    ax1 = fig.add_subplot(311)
    ax2 = fig.add_subplot(312)
    ax3 = fig.add_subplot(313)
    
    ax1.plot(SollWerte,'r-', IstWerte,'b-')
    ax2.plot(StellWerte,'g-')
    ax3.plot(IstWerte,'b-')
    
    ax1.set_title('Sollwert')
    ax2.set_title('Stellwert')
    ax3.set_title('Istwert')
    
    ax1.axis([0,Iterations,0,30])
    ax2.axis([0,Iterations,-20,50])
    ax3.axis([0,Iterations,-0,30])
    
    plt.show()


"""call main().""" 
if __name__ == "__main__":
    main()
    
