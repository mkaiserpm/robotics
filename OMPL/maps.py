'''
Created on 11.04.2017

@author: mario
'''
import os
import matplotlib.image as img
from scipy import ndimage

MAPPATH = os.path.join(os.path.dirname(__file__),"maps")

class GridMap():
    def __init__(self,filename,collfreevalue = 0.):
        '''
        Import image file given as filename
        Assumes PIL library / Pillow 
        '''
        mapfilename = os.path.join(MAPPATH,filename)
        self.mapimage = img.imread(mapfilename)
        self.dimy, self.dimx = self.mapimage.shape
        self.collfreevalue = collfreevalue
        self.obstaclemap = self.disttrans_fast()
        self.goaldistmap = None
    
    def show(self, mymap = None):
        #assumes matplotlib is available
        
        self.plot_(plt,mymap)
        #code will stop here untill pyplot is closed manually!
        plt.show()
    
    def plot_(self,plt,mymap = None):
        if not mymap:
            mymap = self.mapimage
        plt.imshow(mymap, cmap = "gray")
        plt.colorbar()
        
    def plotsolution(self,graph):
        self.plot_(plt)
        graph.plot_(plt)
        graph.plot_solutionpath(plt)
        plt.show()
    
    def plotedges(self,graph,node):
        self.plot_(plt)
        graph.plot_edges(plt,node)
        plt.show()
            
    
    def checkcol_pos(self,pos):
        '''
        Checks if pos in in CFree (color is white)
        '''
        if pos[0] >= self.dimx:
            return True
        if pos[1] >= self.dimy:
            return True
        
        mapval = self.mapimage[pos[1],pos[0]]
        if mapval >0.:
            return True
        return False
    
    def check_collisions(self,xlist,ylist):
        '''
        Checks a list of positions given as list xpos and list ypos (must be same size)
        Returns a list of poss (x,y) which are in collision
        '''
        #Get all avlues at given indeces
        #No dimensionality check!
        collvalues = self.mapimage[ylist,xlist]
        #Get indexes where value is 1. (thats a colision)
        collidxs = np.where(collvalues==1.)[0]
        if len(collidxs) > 0:
            xcolls = xlist[collidxs]
            ycolls = ylist[collidxs]
            colltuples = zip(xcolls,ycolls)
        else:
            colltuples = []
        return colltuples
    
    def dimfail(self,checkgridpos,dimval):
        '''
        Helperfunction to support grid index checking with positions
        Returns 
        '''
        if (checkgridpos > (dimval-1) or (checkgridpos < 0)):
            return True
        return False
    
    def dimcheck(self,position):
        '''
        Checks if position is within map limits
        Returns True if OK
        False if fails
        '''
        
        xpos = position[0]
        xdim = self.dimx
        ypos = position[1]
        ydim = self.dimy
        if self.dimfail(xpos,xdim):
            return False
        if self.dimfail(ypos,ydim):
            return False
        return True
        
        
        
    def linefreecheck(self,pos1,pos2,collfreevalue):
        '''
        Input: x,y cell indeces of starting end ending cell
        collfreevalue: the (float) value until when the cell value is considered 
        as free
        
        if positions are out of gridbounds, it is not free
        
        We can return immediately after finding the first collision cell 
        along the path
        '''
        x0 = pos1[0]
        y0 = pos1[1]
        x1 = pos2[0]
        y1 = pos2[1]
        xorig = x0
        yorig = y0
        #check gridbounds
        if (self.dimfail(x0,self.dimx) or self.dimfail(x1,self.dimx)):
            print("linecheck: x-values out of grid bounds!")
            return (xorig,yorig)
        if (self.dimfail(y0,self.dimy) or self.dimfail(y1,self.dimy)):
            print("linecheck: y-values out of grid bounds!")
            return (xorig,yorig)
        
        dx = abs(x1 - x0)    # distance to travel in X
        dy = abs(y1 - y0)    # distance to travel in Y
        if type(dx) != type(1):
            print("Error: float given as x gridpos!")
        if type(dy) != type(1):
            print("Error: float given as y gridpos!")

        if x0 < x1:
            ix = 1           # x will increase at each step
        else:
            ix = -1          # x will decrease at each step
    
        if y0 < y1:
            iy = 1           # y will increase at each step
        else:
            iy = -1          # y will decrease at each step
    
        e = 0                # Current error 
        xprev = x0
        yprev = y0
        for i in range(dx + dy):
            try:
                gridvalue = self.mapimage[y0][x0]
            except IndexError:
                print("Encountered position out of grid bounds!")
                print("MapBound(ydim,xdim) ({},{}), Indexes(y,x) ({},{}) at setp {}".format(self.dimy,self.dimx,y0,x0,i))
                return (xorig,yorig)
            if gridvalue > collfreevalue:
                return (xprev,yprev)
            e1 = e + dy
            e2 = e - dx
            xprev = x0
            yprev = y0
            if abs(e1) < abs(e2):
                # Error will be smaller moving on X
                x0 += ix
                e = e1
            else:
                # Error will be smaller moving on Y
                y0 += iy
                e = e2            
            #dimension check, if we hit the boundaries, it's like hitting a wall
            if self.dimfail(x0,self.dimx) or self.dimfail(y0,self.dimy):
                return (xprev,yprev) 
        if self.checkcol_pos((x0,y0)):
            #print("Linefreecheck error! position found in collision {}".format((x0,y0)))
            return (xprev,yprev)
        return (x0,y0)
    
    def checkcol_lastfreepos(self,pos1,pos2):
        '''
        More complicated check TBD
        besenham algo will be extended for all visited gridsquares along the
        line pos1/pos2
        
        '''
        checkedlinepos =self.linefreecheck(pos1,pos2,self.collfreevalue)
        return checkedlinepos
    
    
    def checkcol_path(self,pos1,pos2):
        '''
        More complicated check TBD
        besenham algo will be extended for all visited gridsquares along the
        line pos1/pos2
        
        '''
        checklinefree =self.linefreecheck(pos1,pos2,self.collfreevalue)
        if checklinefree == pos2: 
            return False
        return True
    
    def disttrans_fast(self):
        mapinv = 1-self.mapimage
        trans = ndimage.distance_transform_edt(mapinv)
        return trans
    
    def genGoalDistMap(self,goalpos):
        if self.dimcheck(goalpos):
            mapgoal = np.ones(self.mapimage.shape)
            mapgoal[goalpos[1],goalpos[0]] = 0.
            self.goaldistmap = ndimage.distance_transform_edt(mapgoal)