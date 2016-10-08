'''
Created on 08.10.2016

@author: mario
'''
import unittest
from BotMover import WheelEncoder, BotDiffWheel


class TestOdometry(unittest.TestCase):


    def setUp(self):
        pass


    def tearDown(self):
        pass


    def testOdoexample(self):
        bot = BotDiffWheel(wheeldist= 4.,
                           wheelradius=2.,
                           xpos = 0.,
                           ypos = 0.,
                           phi = 0.,
                           starttime = 0.)
        
        odo = WheelEncoder(100)
        x,y,phi = odo.getUpdatePos(6,10, bot)
        print("Xupd: %f Yupd: %f Phiupd: %f"%(x,y,phi))
        self.assertAlmostEqual(x,1.0053,delta = 0.00001)
        self.assertAlmostEqual(y, 0., delta=0.00001)
        self.assertAlmostEqual(phi,0.1257, delta=0.0001)


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()