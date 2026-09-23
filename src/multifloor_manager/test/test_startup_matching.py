#!/usr/bin/env python3
"""Known geometric rays generated independently of the matching implementation."""
import math
import unittest
import numpy as np
from multifloor_manager.startup_matching import ScanMatcher


def room(symmetric=False):
    grid = np.zeros((120,160), dtype=np.int16)
    grid[:2,:] = grid[-2:,:] = 100
    grid[:,:2] = grid[:,-2:] = 100
    if not symmetric:
        grid[25:44,50:65] = 100
        grid[70:95,105:115] = 100
    return grid


def raycast(grid, pose, resolution=.05):
    """March independent rays until the first occupied cell (not matcher score)."""
    points=[]
    for angle in np.linspace(-math.pi, math.pi, 360, endpoint=False):
        for distance in np.arange(.05,15.,.015):
            x=pose[0]+distance*math.cos(angle+pose[2])
            y=pose[1]+distance*math.sin(angle+pose[2])
            ix,iy=int(x/resolution),int(y/resolution)
            if not 0<=ix<grid.shape[1] or not 0<=iy<grid.shape[0]:
                break
            if grid[iy,ix] >= 65:
                points.append((distance*math.cos(angle),distance*math.sin(angle)))
                break
    return np.array(points)


class MatchingTest(unittest.TestCase):
    def test_arbitrary_nonhome_positions(self):
        grid=room()
        for expected in ((6.26,1.11,-1.27),(3.71,4.79,2.65)):
            with self.subTest(expected=expected):
                match=ScanMatcher(grid,.05,(0,0,0)).search(raycast(grid,expected))
                self.assertTrue(match.unique, match)
                self.assertLess(math.hypot(match.pose[0]-expected[0],match.pose[1]-expected[1]),.10)
                self.assertLess(abs(math.atan2(math.sin(match.pose[2]-expected[2]),math.cos(match.pose[2]-expected[2]))),.04)

    def test_rectangle_does_not_certify_opposite_corner(self):
        grid=room(True)
        match=ScanMatcher(grid,.05,(0,0,0)).search(raycast(grid,(1.27,2.16,.72)))
        self.assertGreater(match.hit_fraction,.9)
        self.assertFalse(match.unique,match)

    def test_rotated_map_origin(self):
        grid=room()
        pose=(6.26,1.11,-1.27)
        a=.4
        match=ScanMatcher(grid,.05,(-2.,5.,a)).search(raycast(grid,pose))
        expected=(pose[0]*math.cos(a)-pose[1]*math.sin(a)-2,pose[0]*math.sin(a)+pose[1]*math.cos(a)+5)
        self.assertTrue(match.unique,match)
        self.assertLess(math.hypot(match.pose[0]-expected[0],match.pose[1]-expected[1]),.10)

    def test_unknown_space_and_wall_pose_rejected(self):
        grid=room();grid[40:50,30:40]=-1
        matcher=ScanMatcher(grid,.05,(0,0,0))
        scores,hits=matcher.score([(1.75,2.25,0),(2.9,1.7,0)],np.ones((30,2)))
        self.assertTrue(np.all(scores<0))
        self.assertTrue(np.all(hits==0))

    def test_cancellation_preempts_search(self):
        grid=room()
        with self.assertRaises(InterruptedError):
            ScanMatcher(grid,.05,(0,0,0)).search(raycast(grid,(1.,1.,0)),lambda:True)

    def test_missing_scan_returns(self):
        with self.assertRaises(ValueError):
            ScanMatcher(room(),.05,(0,0,0)).search(np.full((90,2),np.nan))


if __name__=='__main__':
    unittest.main()
