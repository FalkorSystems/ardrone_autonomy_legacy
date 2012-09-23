#!/usr/bin/env python
"""
Tracker classes must implement track( frame ) which returns a tuple (x,y,area)
if the tracker does not find an object then it will return None
"""

import numpy as np
import math
import cv2

lk_params = dict( winSize = ( 30, 30 ),
                  maxLevel = 2,
                  criteria = ( cv2.TERM_CRITERIA_EPS |
                               cv2.TERM_CRITERIA_COUNT, 10, 0.03 ) )

feature_params = dict( maxCorners = 50,
                       qualityLevel = 0.3,
                       minDistance = 10,
                       blockSize = 10 )

class dummyTracker:
    def track( self, frame ):
        (y,x,n) = frame.shape
        cv2.imshow( 'Dummy', frame )
        return( 50, 50, 50 )

class LkTracker:
    def __init__(self):
        self.track_len = 10
        self.detect_interval = 5
        self.tracks = []
        self.frame_idx = 0
        self.frame = None
        self.mouseDown = False

        self.userRect = None
        cv2.namedWindow( 'LKTracker' )
        cv2.cv.SetMouseCallback( 'LKTracker', self.on_mouse, None )

    def on_mouse( self, event, x, y, flags, param ):
        if self.frame is None:
            return

        if event == cv2.cv.CV_EVENT_LBUTTONDOWN:
            self.mouseDown = True
            self.userRect = np.int32( ( ( x, y ), ( x, y ) ) )

        elif event == cv2.cv.CV_EVENT_MOUSEMOVE and self.mouseDown == True:
            xmin = min( self.userRect[0,0], self.userRect[1,0], x)
            xmax = max( self.userRect[0,0], self.userRect[1,0], x)
            ymin = min( self.userRect[0,1], self.userRect[1,1], y)
            ymax = max( self.userRect[0,1], self.userRect[1,1], y)
            self.userRect = np.int32( ( ( xmin, ymin ), ( xmax, ymax ) ) )
            
        elif event == cv2.cv.CV_EVENT_LBUTTONUP:
            self.mouseDown = False
            self.pickFeatures()
            self.initCamshift()
            self.userRect = None

    def initCamshift(self):
        x0,y0,x1,y1 = (self.userRect[0,0],
                       self.userRect[0,1],
                       self.userRect[1,0],
                       self.userRect[1,1])

        hsv_roi = self.hsv[ y0:y1, x0:x1 ]
        mask_roi = self.hsv_mask[ y0:y1, x0:x1 ]

        hist = cv2.calcHist( [hsv_roi], [0], mask_roi, [16], [0,180] )
        cv2.normalize( hist, hist, 0, 255, cv2.NORM_MINMAX )
        self.hist = hist.reshape(-1)
        self.track_window = ( x0, y0, x1-x0, y1-y0 )
        self.showHist()

    def getCamShift(self):
        prob = cv2.calcBackProject( [self.hsv], [0], self.hist, [0,180], 1 )
        prob &= self.hsv_mask
        term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
        track_box, self.track_window = cv2.CamShift( prob, self.track_window, 
                                                     term_crit )
#        cv2.imshow( 'Back Projection', prob )
        return track_box
    
    def showHist(self):
        bin_count = self.hist.shape[0]
        bin_w = 24
        img = np.zeros((256, bin_count*bin_w, 3), np.uint8)
        for i in xrange(bin_count):
            h = int(self.hist[i])
            cv2.rectangle(img, (i*bin_w+2, 255), ((i+1)*bin_w-2, 255-h), (int(180.0*i/bin_count), 255, 255), -1)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
#        cv2.imshow('hist', img)
        
    def pickFeatures(self):
        mask = np.zeros_like( self.frame_gray )
        cv2.rectangle( mask, tuple( self.userRect[0] ), tuple( self.userRect[1] ), 255, -1 )
#        cv2.imshow( 'userMask', mask )

        p = cv2.goodFeaturesToTrack( self.frame_gray, mask = mask, **feature_params )
        if p is not None:
            self.tracks = [ [ (x,y) ] for x, y in np.float32(p).reshape(-1, 2) ]

    def equalizeHist(self, frame):
        splits = cv2.split( frame )
        equalized = map( cv2.equalizeHist, splits )
        return cv2.merge( equalized )

    def initializeFrame( self, frame ):
        self.frame = cv2.pyrDown( frame )
        self.hsv = cv2.cvtColor( self.frame, cv2.COLOR_BGR2HSV )
        self.hsv = self.equalizeHist( self.hsv )
        self.hsv_mask = cv2.inRange( self.hsv, np.array((0.0, 33.0, 33.0)),
                                     np.array((180.0,254.,254.)) )

        # dilate
        self.hsv_mask = cv2.dilate( self.hsv_mask, None, iterations = 5 )

#            self.frame_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        hsv_split = cv2.split( self.hsv )
        self.frame_gray = hsv_split[0]
#        cv2.imshow( 'sat', hsv_split[1] )
#        cv2.imshow( 'vol', hsv_split[2] )
        
        cv2.imshow( 'gray', self.frame_gray )
#        cv2.imshow( 'hsv_mask', self.hsv_mask )

    def filterOutliers( self, tracks ):
        pts = np.int32( [ tr[-1] for tr in tracks ] )
        if pts.size < 10: # 5 pts (10 bc x/y)
            return tracks

        x_median = np.median( pts[:,0] )
        y_median = np.median( pts[:,1] )

        distances = ( np.square(pts[:,0] - x_median) +
                      np.square(pts[:,1] - y_median) )
        distance_median = np.median( distances )

        new_tracks = [ tr for (i,tr) in enumerate( tracks )
                       if distances[i] < distance_median * 9 ]

        return new_tracks

    def runOpticalFlow( self ):
        if len(self.tracks) > 0:
            img0, img1 = self.prev_gray, self.frame_gray
            p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 1, 2 )
            p1, st, err = cv2.calcOpticalFlowPyrLK( img0, img1, p0, None, **lk_params )
            p0r, st, err = cv2.calcOpticalFlowPyrLK( img1, img0, p1, None, **lk_params )
            
            d = abs( p0-p0r ).reshape(-1, 2).max(-1)
            good = d < 1
            new_tracks = []
            for tr, (x, y), good_flag in zip( self.tracks, p1.reshape(-1,2), good ):
                if not good_flag:
                    continue
                tr.append((x, y))
                if len(tr) > self.track_len:
                    del tr[0]
                    
                new_tracks.append(tr)

            
                
            self.tracks = self.filterOutliers( new_tracks )
    
    def reDetect( self ):
        if self.frame_idx % self.detect_interval == 0 and len(self.tracks) > 0:
            mask_camshift = np.zeros_like( self.frame_gray )
            mask_tracked = np.zeros_like( self.frame_gray )

            ellipse_camshift = self.getCamShift()
            pts = np.int32( [ [tr[-1]] for tr in self.tracks ] )

            if len(pts) > 5:
                ellipse_tracked = cv2.fitEllipse( pts )
            else:
                ellipse_tracked = cv2.minAreaRect( pts )

            boundingRect = cv2.boundingRect( pts )
            # x0,y0,x1,y1 = ( boundingRect[0] - int(boundingRect[2] * 0.05),
            #                 boundingRect[1] - int(boundingRect[3] * 0.05),
            #                 boundingRect[0] + int(boundingRect[2] * 1.05),
            #                 boundingRect[1] + int(boundingRect[3] * 1.05))
            x0,y0,x1,y1 = ( boundingRect[0] - 1,
                            boundingRect[1] - 1,
                            boundingRect[0] + boundingRect[2] + 1,
                            boundingRect[1] + boundingRect[3] + 1 )
            
            if len( pts ) > 2:
                cv2.rectangle( mask_tracked, (x0,y0), (x1,y1), 255, -1 )

            cv2.ellipse( mask_camshift, ellipse_camshift, 255, -1 )
            cv2.ellipse( mask_tracked, ellipse_tracked, 255, -1 )

#            cv2.imshow( 'mask_camshift', mask_camshift )
#            cv2.imshow( 'mask_tracked', mask_tracked )

            mask = mask_camshift & mask_tracked

            for x, y in [np.int32(tr[-1]) for tr in self.tracks]:
                cv2.circle(mask, (x,y), 5, 0, -1 )

#            cv2.imshow( 'mask', mask )
            p = cv2.goodFeaturesToTrack( self.frame_gray, mask = mask, **feature_params )
            if p is not None:
                for x, y in np.float32(p).reshape(-1, 2):
                    self.tracks.append([(x,y)])

    def drawAndGetTrack( self ):
        vis = self.equalizeHist( self.frame.copy() )
        
        if len(self.tracks) > 0:
            cv2.polylines( vis, [ np.int32(tr) for tr in self.tracks ], False, ( 0, 255, 0 ) )
            pts = np.int32( [ [tr[-1]] for tr in self.tracks ] )

            for [(x,y)] in pts:
                cv2.circle( vis, (x, y), 2, (0, 255, 0), -1 )

            if len( pts ) > 2:
                boundingRect = cv2.boundingRect( pts )
                x0,y0,x1,y1 = ( boundingRect[0], boundingRect[1],
                                boundingRect[0] + boundingRect[2],
                                boundingRect[1] + boundingRect[3] )
                area = boundingRect[3] * boundingRect[2]
                cx,cy = ( boundingRect[0] + boundingRect[2]/2,
                          boundingRect[1] + boundingRect[3]/2 )

                cv2.rectangle( vis, (x0,y0), (x1,y1), (0,255,255), 3, 8, 0 )
                cv2.circle( vis, (cx,cy), 5, (0,255,255), -1, 8, 0 )

        if self.userRect != None:
            cv2.rectangle( vis, tuple( self.userRect[0] ), tuple( self.userRect[1] ), ( 255, 255, 255 ) )

        cv2.imshow( 'LKTracker', vis )
        if len( self.tracks ) > 2:
            imageSize = self.frame.shape
            imageArea = imageSize[0]*imageSize[1]

            xRel = cx*100/imageSize[1]
            yRel = cy*100/imageSize[0]
            areaRel = math.sqrt( float( area ) / float( imageArea ) ) * 100
            return xRel,yRel,areaRel
        else:
            return None

    def track(self, frame):
        self.initializeFrame( frame )
        self.runOpticalFlow()
        self.reDetect()
        trackData = self.drawAndGetTrack()

        self.frame_idx += 1
        self.prev_gray = self.frame_gray

        return trackData

def main():
    import sys
    try: video_src = sys.argv[1]
    except: video_src = 0

    tracker = LkTracker()
    cam = cv2.VideoCapture( video_src )

    while True:
        ret, frame = cam.read()
        if ret:
            trackData = tracker.track( frame )

        if trackData:
            print trackData

        ch = 0xFF & cv2.waitKey(1)
        if ch == 27:
            break

if __name__ == '__main__':
    main()



                    
                
                
                 
            
    
