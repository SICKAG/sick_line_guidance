# MLS_Measurement.msg defines a single measurement of a SICK MLS.
# See operation instructions for details (www.sick.com/mls).
#

# Header with sequence id, timestamp of the measurement and frame id 
Header header

# Array of measured positions for up to 3 lines.
# Each position is the distance to the center of a line (line center point) in meter.
# More than one line is detected in case of junctions.
float32[] position      # distance to the line center point [m]

# LCP-flags (signs and line assignment)
uint8 lcp               # flags (signs and assignment, see chap. 8 of operation instructions)

# Detection status
uint8 status            # status (see chap. 8 of operation instructions)

# Error register
uint8 error             # error register (0x1001, value 0 = okay, see chap. 8 of operation instructions)

