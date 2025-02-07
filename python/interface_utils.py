from numpy import *
from matplotlib.pyplot import *

####### PLOTTING FUNCTION #########
def plotLatest(fname):
    try:
        df = open(fname,'r')

        r = []
        dr = []
        st = []
        ds = []
        U = []
        t = []
        for line in df:
            # print(line)
            linesplit = line.strip().split(',')
            # print(len(linesplit))
            if(len(linesplit)==8):#valid line if 8 columns
                #print(linesplit)
                t=append(t,float(linesplit[0]))
                dr=append(dr,float(linesplit[2]))
                r=append(r,float(linesplit[3]))
                ds=append(ds,float(linesplit[4]))
                st=append(st,float(linesplit[5]))
                U=append(U,float(linesplit[6]))
                #print(t[-1],dr[-1],r[-1],ds[-1],st[-1],U[-1])
        figure()
        subplot(311)
        plot(t,U,'k')
        ylabel('Speed (m/s)')
        subplot(312)
        plot(t,dr,'r',t,r,'k')
        ylabel('Roll (rad)')
        legend(['desired','actual'])
        subplot(313)
        plot(t,ds,'r',t,st,'k')
        ylabel('steer (rad)')
        legend(['desired','actual'])
        show()
    except:
        pass

if __name__=='__main__':
    plotLatest('data/data_20240726-122515.txt')
