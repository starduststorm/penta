#!/Applications/Kicad/kicad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
from pathlib import Path
from math import *
import cmath
import argparse
from urllib.request import getproxies

parser = argparse.ArgumentParser()
parser.add_argument('-p', "--path", action='store', required=True, help="Path to kicad_pcb file")
parser.add_argument('-L', "--do-layout", action='store_true', required=False, help="Run main layout")

parser.add_argument('--draw-poly', action='store', help='Draw a polygon with format "sides,radius,layer"')
parser.add_argument('--draw-wave', action='store', help='Draw a wave with format "startx,starty,endx,endy,cycles,amplitude,segments,layer"')

parser.add_argument('--delete-all-traces', action='store_true', help='Delete All Traces in the pcbnew and then exit')
parser.add_argument('--delete-all-drawings', action='store_true', help='Delete all lines & texts in the pcbnew and then exit')
parser.add_argument('--delete-short-traces', action='store_true', help='Delete traces of zero or very small length and exit')
parser.add_argument('--dry-run', action='store_true', help='Don\'t save results')
parser.add_argument('-v', dest='verbose', action='count', help="Verbose")
parser.add_argument('--skip-traces', action='store_true', help='Don\'t add traces')
parser.add_argument('--hide-pixel-labels', action='store_true', help="For all modules like D*, set reference text to hidden")
args = parser.parse_args()

# sys.path.insert(0, "/Applications/Kicad/kicad.app/Contents/Frameworks/python/site-packages/")
sys.path.insert(0, "/Applications/Kicad/kicad.app/Contents/Frameworks/Python.framework//Versions/Current/lib/python3.9/site-packages")
sys.path.append("/usr/local/lib/python3.9/site-packages")

import pcbnew
from kicad.point import Point
IU_PER_MM = pcbnew.PCB_IU_PER_MM

def sign(x):
  return 1 if x >= 0 else -1

def circle_pt(theta, radius):
  return Point(radius*cos(theta), radius*sin(theta))

class Point(object):
  @classmethod
  def fromWxPoint(cls, wxPoint):
    return Point(wxPoint.x / IU_PER_MM, wxPoint.y / IU_PER_MM)

  @classmethod
  def fromVector2i(cls, vector):
    return Point(vector.x / IU_PER_MM, vector.y / IU_PER_MM)

  @classmethod
  def fromComplex(cls, c):
    return Point(c.real, c.imag)

  def __init__(self, arg1, arg2=None):
    if arg2 is not None:
      self.x = arg1
      self.y = arg2
    elif type(arg1) == pcbnew.wxPoint:
      p = Point.fromWxPoint(arg1)
      self.x = p.x
      self.y = p.y
    elif type(arg1) == pcbnew.VECTOR2I:
      p = Point.fromVector2i(arg1)
      self.x = p.x
      self.y = p.y
    elif type(arg1) == complex:
      p = Point.fromComplex(arg1)
      self.x = p.x
      self.y = p.y
    else:
      self.x = arg1[0]
      self.y = arg1[1]

  def wxPoint(self):
    return pcbnew.wxPoint(self.x * IU_PER_MM , self.y * IU_PER_MM)
  
  def vector2i(self):
    return pcbnew.VECTOR2I_MM(self.x, self.y)

  def point2d(self):
    return Point(x,y)

  def translate(self, vec):
    self.x += vec.x
    self.y += vec.y

  def translated(self, vec):
    return Point(self.x+vec.x, self.y+vec.y)

  def polar_translated(self, distance, angle):
    vec = distance * cmath.exp(angle * 1j)
    return self.translated(Point.fromComplex(vec))

  def __getattr__(self, attr):
    if attr == "theta":
      if self.x == 0:
        return pi/2 if self.y > 0 else 3*pi/2
      theta = atan(self.y/self.x)
      if self.x > 0 and self.y > 0:
        return theta
      elif self.x > 0 and self.y < 0:
        return 2*pi + theta
      else:
        return pi + theta
    elif attr == "radius":
      return sqrt(self.x**2 + self.y**2)
    else:
      super().__getattr__(attr)
  
  def __setattr__(self, attr, val):
    if attr == "theta":
      r = self.radius
      self.x = r * cos(val)
      self.y = r * sin(val)
    elif attr == "radius":
      theta = self.theta
      self.x = val * cos(theta)
      self.y = val * sin(theta)
    else:
      super().__setattr__(attr, val)

  def __getitem__(self, i):
    if i == 0:
      return self.x
    if i == 1:
      return self.y
    raise IndexError("index out of range")

  def distance_to(self, other):
    other = Point(other)
    return sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)


  def lerp_to(self, other, fraction):
    x = fraction * (other.x - self.x) + self.x
    y = fraction * (other.y - self.y) + self.y
    return Point(x,y)

  def __neg__(self):
    return Point(-self.x, -self.y)

  def __str__(self):
    return "(%0.2f, %0.2f)" % (self.x, self.y)

  def __add__(self, oth):
    oth = Point(oth)
    return Point(self.x + oth.x, self.y + oth.y)

  def __sub__(self, oth):
    oth = Point(oth)
    return Point(self.x - oth.x, self.y - oth.y)

  def __truediv__(self, scalar):
    return Point(self.x/scalar, self.y/scalar)
  
  def __eq__(self, oth):
    return self.x == oth.x and self.y == oth.y

  def __complex__(self):
    return self.x + self.y * 1j


###############################################################################

  
class KiCadPCB(object):
  def __init__(self, path):
    self.pcb_path = os.path.abspath(path)
    self.board = pcbnew.LoadBoard(self.pcb_path)
    tracks = self.board.GetTracks()
    
    self.numlayers = pcbnew.PCB_LAYER_ID_COUNT

    self.layertable = {}
    for i in range(self.numlayers):
      self.layertable[self.board.GetLayerName(i)] = i

    self.netcodes = self.board.GetNetsByNetcode()
    self.netnames = {}  
    for netcode, net in self.netcodes.items():
      self.netnames[net.GetNetname()] = net

  def save(self):
    print("Saving...")
    backup_path = self.pcb_path + ".layoutbak"
    if os.path.exists(backup_path):
      os.unlink(backup_path)
    os.rename(self.pcb_path, backup_path)
    print("  Backing up '%s' to '%s'..." % (self.pcb_path, backup_path))
    assert self.board.Save(self.pcb_path)
    print("Saved!")

  def deleteEdgeCuts(self):
    self.deleteAllDrawings(layer='Edge.Cuts')

  def deleteAllTraces(self):
    tracks = self.board.GetTracks()
    for t in tracks:
      print("Deleting track {}".format(t))
      self.board.Delete(t)
  
  def deleteAllDrawings(self, layer=None):
    for drawing in self.board.GetDrawings():
        if layer is None or layer == drawing.GetLayerName():
          print("Deleting drawing {}".format(drawing))
          self.board.Delete(drawing)
  
  def deleteShortTraces(self):
    print("FIXME: disabled because it removes vias")
    exit(1)
    tracks = board.GetTracks()
    for t in tracks:
      start = t.GetStart()
      end = t.GetEnd()
      length = sqrt((end.x - start.x)**2 + (end.y - start.y)**2)
      if length < 100: # millionths of an inch
        print("Deleting trace of short length {}in/1000000".format(length))
        board.Delete(t)
      elif args.hide_pixel_labels:
        hid = 0
        for m in board.m_Modules:
          if m.GetReference().startswith('D'):
            if m.Reference().IsVisible():
              m.Reference().SetVisible(False)
              hid+=1
        print("Hid %i reference labels" % hid)

  def delete_tracks_connected_to_track(self, track):
    track_start = Point.fromVector2i(track.GetStart())
    track_end = Point.fromVector2i(track.GetEnd())

    tracks = self.board.GetTracks()
    for t in tracks:
      t_start = Point.fromVector2i(t.GetStart())
      t_end = Point.fromVector2i(t.GetEnd())
      
      thresh = .0001
      d1 = track_start.distance_to(t_start)
      d2 = track_start.distance_to(t_end)
      d3 = track_end.distance_to(t_start)
      d4 = track_end.distance_to(t_end)

      if d1 < thresh or d2 < thresh or d3 < thresh or d4 < thresh:
        print("    Deleting connected track of length", t_start.distance_to(t_end))
        self.board.Delete(t)

  def delete_tracks_for_module(self, module): 
    tracks = self.board.GetTracks()
    for pad in module.Pads():
      pad_position = Point.fromVector2i(pad.GetPosition())
      print("  Deleting old tracks for module %s pad %s net %s" % (module.GetReference(), pad.GetPadName(), pad.GetNetname()))
      for track in tracks:
        thresh = 0.0001
        d1 = pad_position.distance_to(track.GetStart())
        d2 = pad_position.distance_to(track.GetEnd())
        if d1 < thresh or d2 < thresh:
          self.delete_tracks_connected_to_track(track)
        


          # vias.append(self.board.GetViaByPosition(track_start.vector2i()))
          # vias.append(self.board.GetViaByPosition(track_end.vector2i()))


          # FIXME: Deleting a via crashes inside kicad API
          # should be able to find and delete zero-length traces to get these??

          # print("vias matching track start: %s" % str())
          # print("vias matching track end: %s" % str(board.GetViaByPosition(track_end.wxPoint())))
          # for via in vias:
          #  if via:
          # for via in board.vias:
          #   if track_start.distance_to(via.position) < thresh or track_end.distance_to(via.position) < thresh:
          #    print("Deleting old via connected to track on net %s" % (track_net.GetNetname(), ))
          #    board.Delete(via)

  # Drawing Tools
  def draw_segment(self, start, end, layer='F.Silkscreen', width=0.15):
    line = pcbnew.PCB_SHAPE()
    self.board.Add(line)
    line.SetShape(pcbnew.S_SEGMENT)
    line.SetStart(start.vector2i())
    line.SetEnd(end.vector2i())
    print("LINE from (%f, %f) to (%f, %f)" % (start.x, start.y, end.x, end.y))
    line.SetLayer(self.layertable[layer])
    line.SetWidth(int(width * IU_PER_MM))
    return line
  
  def draw_circle(self, center, radius, layer='F.Silkscreen', width=0.15):
      circle = pcbnew.PCB_SHAPE()
      self.board.Add(circle)
      circle.SetShape(pcbnew.SHAPE_T_CIRCLE)
      print("CENTER: ", center, type(center))
      print("CENTER: ", center.wxPoint(), type(center.vector2i()))
      circle.SetCenter(center.vector2i())
      circle.SetArcAngleAndEnd(pcbnew.EDA_ANGLE(360, pcbnew.DEGREES_T))
      circle.SetEnd((center + Point(radius,0)).vector2i()) # Sets radius.. somehow
      circle.SetLayer(self.layertable[layer])
      circle.SetWidth(int(width * IU_PER_MM))
      return circle
  
  # def draw_arc_1(self, center, radius, start_angle, stop_angle,
  #                 layer='F.Silkscreen', width=0.15):
      
  #     print("Drawing arc with center at (%f,%f), radius: %f, angle %f -> %f" % (center.x, center.y, radius, start_angle, stop_angle))
  #     arcStartPoint = radius * cmath.exp(start_angle * 1j)
  #     arcStartPoint = center.translated(Point.fromComplex(arcStartPoint))
  #     print("arcStartPoint %s" % str(arcStartPoint));
  #     angle = stop_angle - start_angle
  #     arc = pcbnew.PCB_ARC(self.board)
  #     self.board.Add(arc)
  #     arc.SetShape(pcbnew.S_ARC)
  #     arc.SetCenter(center.wxPoint())
  #     arc.SetArcStart(arcStartPoint.wxPoint())
  #     arc.SetAngle(angle * 180/pi * 10)
  #     arc.SetLayer(self.layertable[layer])
  #     arc.SetWidth(int(width * IU_PER_MM))
  #     return arc

  # def draw_arc_2(self, center, arcStartPoint, totalAngle,
  #                 layer='F.Silkscreen', width=0.15):
  #     print("Drawing arc with center: (%f, %f), arc point at (%f,%f), total angle %f" % (center.x, center.y, arcStartPoint.x, arcStartPoint.y, totalAngle))
  #     arc = pcbnew.PCB_ARC(self.board)
  #     self.board.Add(arc)
  #     arc.SetShape(pcbnew.S_ARC)
  #     arc.SetCenter(center.wxPoint())
  #     arc.SetArcStart(arcStartPoint.wxPoint())
  #     arc.SetAngle(totalAngle * 180/pi * 10)
  #     arc.SetLayer(self.layertable[layer])
  #     arc.SetWidth(int(width * IU_PER_MM))
  #     return arc

  def add_copper_trace(self, start, end, net):
    track = pcbnew.PCB_TRACK(self.board)
    track.SetStart(start)
    track.SetEnd(end)
    track.SetLayer(self.layertable["F.Cu"])
    track.SetWidth(int(.25 * 10**6))
    track.SetNet(net)
    self.board.Add(track)
    print("  copper from {} to {} on net {}".format(start, end, net.GetNetname()))
    return track

  def add_via(self, position, net):
    via = pcbnew.PCB_VIA(self.board)
    via.SetPosition(position)
    via.SetViaType(pcbnew.VIATYPE_BLIND_BURIED)
    via.SetLayerPair(self.layertable["F.Cu"], self.layertable["In1.Cu"])
    via.SetDrill(int(0.254 * IU_PER_MM))
    via.SetWidth(int(0.4064 * IU_PER_MM))
    via.SetNet(net)
    print("Adding via on net %s at %s" % (net.GetNetname(), str(position)))
    self.board.Add(via)


###############################################################################


class PCBLayout(object):
  center = Point(100,100)
  edge_cut_line_thickness = 0.05
  pixel_pad_map = {"1": "4"}
  pixel_pad_names = {"GND": "3", "+5V": "2"}
  
  groundViaAngle = 3*pi/4
  groundViaDistance = 0.5
  vccTraceDistance = 0.2
  vccTraceAngle = -pi/4

  ####

  def __init__(self, path):
    self.kicadpcb = KiCadPCB(path)
    self.board = self.kicadpcb.board

  prev_series_pixel = None
  pixel_prototype = None
  series_pixel_count = 0

  def drawSegment(self, start, end, layer='F.Silkscreen', width=0.15):
    if type(start) is tuple:
      start = Point(start[0], start[1])
    if type(end) is tuple:
      end = Point(end[0], end[1])
    self.kicadpcb.draw_segment(self.center+start, self.center+end, layer, width)

  def seriesPixelDiscontinuity(self):
    self.prev_series_pixel = None

  def drawPoly(self, polySpec):
    def regularPolygonVertices(n_sides, radius):
      angle = 2*pi / n_sides
      vertices = []
      for i in range(n_sides):
        theta = angle * i + 2*pi/n_sides/2
        x = radius * cos(theta)
        y = radius * sin(theta)
        vertices.append((x, y))
      return vertices

    sides, radius, layer = polySpec.split(',')
    sides = int(sides)
    radius = float(radius)

    vertices = regularPolygonVertices(sides,radius)
    for i in range(len(vertices)):
      x1,y1 = vertices[i]
      x2,y2 = vertices[i+1-len(vertices)]
      assert(x1!= x2 or y1 != y2)
      self.drawSegment((x1, y1), (x2, y2), layer)

  def drawWave(self, waveSpec):
    import cmath
    def reorder(a,b):
      if a > b:
        return b,a
      return a,b
    def mkfloat(*args):
      return (float(a) for a in args)

    startx,starty,endx,endy,cycles,amplitude,segments,layer = waveSpec.split(',')
    startx,starty,endx,endy,cycles,amplitude = mkfloat(startx,starty,endx,endy,cycles,amplitude)
    segments = int(segments)
    cycles = 2*pi*cycles
    p1,p2 = reorder((startx,starty), (endx, endy)) # reduce to two quandrants
    p1 = Point(p1)
    p2 = Point(p2)

    print("Draw Wave from (",startx, ",",starty,") to (",endx,",",endy,")")

    prevPoint = None
    for i in range(segments+1):
      xp = i / float(segments) * (p2.x-p1.x)
      yp = -amplitude * cos(i / float(segments) * cycles)
      angle = atan((p2.y-p1.y)/(p2.x-p1.x))

      rotated = Point((xp+yp*1j) * cmath.exp(angle*1j)) + p1
      if prevPoint is not None:
        self.drawSegment(prevPoint,rotated,layer,0.05)
      prevPoint = rotated

  ### ------------------------------------------------------- ###

  def get5VTraceEnd(self, pad, orientation):
    return Point(pad.GetPosition()).polar_translated(self.vccTraceDistance, self.vccTraceAngle-orientation)

  def connectPixels(self, fromPixel, toPixel):
    if not args.skip_traces:
      for pad in toPixel.Pads():
        if pad.GetPadName() == self.pixel_pad_names['+5V']:
          # connect +5V from previous
          prevPad = [pad for pad in fromPixel.Pads() if pad.GetPadName() == self.pixel_pad_names['+5V']][0]
          prevEnd = self.get5VTraceEnd(prevPad, fromPixel.GetOrientation().AsRadians())
          end = self.get5VTraceEnd(pad, toPixel.GetOrientation().AsRadians())
          self.kicadpcb.add_copper_trace(prevEnd.vector2i(), end.vector2i(), pad.GetNet())

      # Add data tracks from the previous pixel
      for prev_pad in fromPixel.Pads():
        if prev_pad.GetPadName() in self.pixel_pad_map:
          # tracks = self.kicadpcb.board.TracksInNet(prev_pad.GetNet().GetNetCode())
          # if tracks:
          #   # skip pad, already has traces
          #   if args.verbose:
          #     print("Skipping pad, already has {} tracks: {}".format(len(tracks), prev_pad.GetPadName()))
          #   continue

          # for net in board.TracksInNet(prev_pad.GetNet().GetNet()):
          #   board.Delete(t)

          # then connect the two pads
          for pad in toPixel.Pads():
            # print("    pad name:", pad.GetPadName())
            if pad.GetPadName() == self.pixel_pad_map[prev_pad.GetPadName()]:
              start = prev_pad.GetPosition()
              end = pad.GetPosition()
              print("Adding track from pixel {} pad {} to pixel {} pad {}".format(fromPixel.GetReference(), prev_pad.GetPadName(), toPixel.GetReference(), pad.GetPadName()))
              self.kicadpcb.add_copper_trace(start, end, pad.GetNet())

  def placePixel(self, reference, point, orientation, allowOverlaps=True, alignOverlaps=True):
    print("Place ", reference)
    
    if self.pixel_prototype is None:
      print("Loading pixel footprint...")
      self.pixel_prototype = pcbnew.FootprintLoad(str(Path(__file__).parent.joinpath('kicad_footprints.pretty').resolve()), "ws2812b_1010")
      self.pixel_prototype.SetLayer(self.kicadpcb.layertable["F.Cu"])

    absPoint = self.center + point

    if not allowOverlaps:
      overlapThresh = 0.8
      for fp in self.kicadpcb.board.GetFootprints():
        if not fp.GetReference().startswith("D"):
          continue
        # print(fp, Point(fp.GetPosition()), fp.GetOrientation())
        fpPoint = Point(fp.GetPosition())
        fpOrientation = fp.GetOrientation().AsRadians()
        if absPoint.distance_to(fpPoint) < overlapThresh:
          if alignOverlaps:
            # the pixel overlaps with a pixel already on the board. we'll adjust it to 'flow' more with the new attemped pixel
            newPt = (fpPoint+absPoint)/2
            orientationTweakSign = 1 if abs(fpOrientation%(pi/2) - orientation%(pi/2)) > pi/4 else -1
            newOrientation = fpOrientation + (fpOrientation%(pi/2) - orientation%(pi/2)) / 2 * orientationTweakSign
            print("Pixel", reference, "overlaps", fp.GetReference(), "first", fpPoint, "@", fpOrientation, "second", absPoint, "@", orientation, " => ", newPt, "@", newOrientation)
            
            movedPads = [pad.GetPosition() for pad in fp.Pads()]
            
            fp.SetOrientation(pcbnew.EDA_ANGLE(180/pi*newOrientation, pcbnew.DEGREES_T))
            fp.SetPosition(newPt.vector2i())

            # adjust any traces that were connected to this pixel
            for padNum in range(len(movedPads)):
              for track in self.kicadpcb.board.GetTracks():
                if Point(track.GetStart()).distance_to(movedPads[padNum]) < 0.1:
                  track.SetStart(fp.Pads()[padNum].GetPosition())
                elif Point(track.GetEnd()).distance_to(movedPads[padNum]) < 0.1:
                  track.SetEnd(fp.Pads()[padNum].GetPosition())
          else:
            print("Skipping pixel", reference, ", due to overlap")
          return None

    print("Placing pixel %s at point %s (relative center %s) orientation %f" % (reference, point, self.center, orientation));
    pixel = pcbnew.FOOTPRINT(self.pixel_prototype)
    pixel.SetPosition((self.center + point).vector2i())
    
    pixel.SetReference(reference)
    # module.position = (point + self.center).point2d()
    pixel.SetOrientation(pcbnew.EDA_ANGLE(orientation, pcbnew.RADIANS_T))
    pixel.Reference().SetVisible(False)
    self.kicadpcb.board.Add(pixel)

    if not args.skip_traces:
      for pad in pixel.Pads():
        if pad.GetPadName() == self.pixel_pad_names['GND']:
          # draw trace outward from ground
          end = Point(pad.GetPosition()).polar_translated(self.groundViaDistance, self.groundViaAngle-orientation).vector2i()
          self.kicadpcb.add_copper_trace(pad.GetPosition(), end, pad.GetNet())

          # add a via
          self.kicadpcb.add_via(end, pad.GetNet())
        elif pad.GetPadName() == self.pixel_pad_names['+5V']:
          # draw trace outward from 5V
          end = self.get5VTraceEnd(pad, orientation)
          self.kicadpcb.add_copper_trace(pad.GetPosition(), end.vector2i(), pad.GetNet())


    return pixel

  def placeSeriesPixel(self, point, orientation, allowOverlaps=True, alignOverlaps=True):
    reference = "D%i" % (self.series_pixel_count+1)
    
    placedPixel = self.placePixel(reference, point, orientation, allowOverlaps, alignOverlaps)
    if placedPixel is not None:
      self.series_pixel_count += 1
      if self.prev_series_pixel is not None:
        self.connectPixels(self.prev_series_pixel, placedPixel)

      self.prev_series_pixel = placedPixel

  def doLayout(self, args):
    if args.delete_all_traces:
      self.kicadpcb.deleteAllTraces()

    if args.delete_all_drawings:
      self.kicadpcb.deleteAllDrawings()

    if args.delete_short_traces:
      self.kicadpcb.deleteShortTraces()
    
    if args.draw_poly:
      self.drawPoly(args.draw_poly)
    if args.draw_wave:
      self.drawWave(args.draw_wave)

    if args.do_layout:
      self.kicadpcb.deleteAllDrawings(layer='F.Silkscreen')
      self.drawEdgeCuts()
      self.placePixels()
      self.decorateSilkScreen()

    if not args.dry_run:
      self.kicadpcb.save()
  
  ### to override
  def drawEdgeCuts(self):
    self.kicadpcb.deleteEdgeCuts()
    pass

  def placePixels(self):
    import re
    for fp in self.kicadpcb.board.GetFootprints():
      if re.match("^D\d+$",fp.GetReference()):
        print("Removing old footprint for pixel %s" % fp.GetReference())
        self.kicadpcb.delete_tracks_for_module(fp)
        self.kicadpcb.board.Delete(fp)
  
  def decorateSilkScreen(self):
    pass


###############################################################################


class LayoutPenta(PCBLayout):
  baseRotate = pi/2
  edgeRadius = 18.7#mm
  circleRadius = 17#mm
  pointCornerSpread = 0.014
  
  def drawEdgeCuts(self):
    super().drawEdgeCuts()
    self.kicadpcb.draw_circle(self.center, self.edgeRadius, 'Edge.Cuts', self.edge_cut_line_thickness)
    
  def placePixels(self):
    super().placePixels()

    # # remove previous tracks
    # modules = dict(zip((m.reference for m in self.kicadpcb.board.modules), (m for m in self.kicadpcb.board.modules)))

    # for module in modules.values():
    #   if module.reference.startswith('D'):
    #     delete_tracks_for_module(module)
    def ft(frm,to):
      inc = -1 if to<frm else 1
      return range(frm, to+inc,inc)
    def flatten(lst):
      flat_list = []
      for item in lst:
          if isinstance(item, (list, tuple, set, range, range)):
              flat_list.extend(flatten(item))
          else:
              flat_list.append(item)
      return flat_list

    def unique(l):
      return len(l) == len(set(l))
    
    # this just serves to reorder the pixel placement from the simple programmatic placement below to a more convenient wiring order
  # placementOrder = [ft(104,108),ft(99,103),ft(118,122),ft(112,109),ft(98,95),ft(84,81),ft(70,66),ft(125,123),ft(113,117),ft(61,65),ft(126,130),ft(76,80),ft(71,75),ft(90,94),ft(85,89),ft(37,60),ft(1,36)]
    placementOrder = [ft(99,103),ft(94,98),ft(113,117),ft(107,104),ft(93,90),ft(79,76),ft(65,61),ft(120,118),ft(108,112),ft(56,60),ft(121,125),ft(71,75),ft(66,70),ft(85,89),ft(80,84),ft(34,55),ft(1,33)]
    placementOrder = flatten(placementOrder)
    print(sorted(placementOrder))
    assert(unique(placementOrder))
    assert(len(placementOrder)==125)
    placementOrderMap = [None] * len(placementOrder)
    for i in range(len(placementOrder)):
      placementOrderMap[placementOrder[i]-1] = i+1

    print("Placing Circle...")
    circlePixels = 55
    for i in range(circlePixels):
      pos = Point(self.circleRadius * cos(2*pi*i/circlePixels + self.baseRotate), self.circleRadius * sin(2*pi*i/circlePixels + self.baseRotate))
      orientation = -2*pi*i/circlePixels + 3*pi/4 + pi
      reference = "D%i" % placementOrderMap[self.series_pixel_count]
      # reference = "D%i" % self.series_pixel_count

      pixel = self.placePixel(reference, pos, orientation + self.baseRotate, allowOverlaps=False)
      if pixel is not None:
        if self.prev_series_pixel is not None:
          self.connectPixels(self.prev_series_pixel, pixel)
        self.prev_series_pixel = pixel
        self.series_pixel_count+=1

    # self.seriesPixelDiscontinuity()

    print("Placing Penta...")

    penta = 5
    for i in range(penta):
      segStart = Point(self.circleRadius * cos(2*pi*(i+self.pointCornerSpread)/penta + self.baseRotate), 
                       self.circleRadius * sin(2*pi*(i+self.pointCornerSpread)/penta + self.baseRotate))
      segEnd = Point(self.circleRadius * cos(2*pi*(i+2-self.pointCornerSpread)/penta + self.baseRotate), 
                     self.circleRadius * sin(2*pi*(i+2-self.pointCornerSpread)/penta + self.baseRotate))
      
      linePixels = 17
      for p in range(linePixels):
        if p == 0 or p == linePixels-1:
          # don't realign overlapping circle pixels
          continue
        pos = segStart.lerp_to(segEnd, p/(linePixels-1))
        orientation = -2*pi*(i+1)/penta - pi/4
        reference = "D%i" % placementOrderMap[self.series_pixel_count]
        # reference = "D%i" % self.series_pixel_count
        pixel = self.placePixel(reference, pos, orientation + self.baseRotate, allowOverlaps=False)
        if pixel is not None:
          if self.prev_series_pixel is not None:
            self.connectPixels(self.prev_series_pixel, pixel)
          self.prev_series_pixel = pixel
          self.series_pixel_count+=1
  
  def insertFootprint(self, name, point, layer="F.Silkscreen"):
    for fp in self.kicadpcb.board.GetFootprints():
      if fp.GetReference() == name:
        print("Removing old footprint for", name)
        self.kicadpcb.board.Delete(fp)
        break
    fp = pcbnew.FootprintLoad(str(Path(__file__).parent.joinpath('kicad_footprints.pretty').resolve()), name)
    fp.SetLayer(self.kicadpcb.layertable[layer])
    fp.SetPosition(point.vector2i())
    fp.SetReference(name)
    fp.Reference().SetVisible(False)
    self.kicadpcb.board.Add(fp)

  def decorateSilkScreen(self):
    super().decorateSilkScreen()

    guides = 5
    for i in range(guides):
      theta = i * 2 * pi/guides
      self.drawSegment((0,0), circle_pt(self.baseRotate + theta, self.edgeRadius), width=0.05)

class Timing(object):
  @classmethod
  def linear(cls, time, start, change, duration):
    return start + change * time/duration
  
  @classmethod
  def easeInOutCubic(cls, time, start, change, duration): 
    t = time / (duration/2)
    return change/2*t*t*t + start if t < 1 else change/2*((t-2)*(t-2)*(t-2) + 2) + start;
    
  @classmethod
  def easeOutCubic(cls, time, start, change, duration):
    time = time/duration
    t2 = time-1
    return change*(t2*t2*t2 + 1) + start

  @classmethod
  def easeInCubic(cls, time, start, change, duration):
    time = time/duration
    return change * time*time*time + start

layout = LayoutPenta(args.path)    

if __name__ == '__main__':
  layout.doLayout(args)

