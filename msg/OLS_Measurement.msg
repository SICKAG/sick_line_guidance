# OLS_Measurement.msg defines a single measurement of a SICK OLS.
# See operation instructions for details (www.sick.com/ols).

# Header with sequence id, timestamp of the measurement and frame id 
Header header

# Array of measured positions for up to 3 lines.
# Each position is the distance to the center of a line (line center point) in meter.
# More than one line is detected in case of junctions.
float32[] position           # distance to the line center point [m]

# Array of up to 3 line widths.
# For each detected line, its width is measured.
float32[] width              # width of line [m]

# Detection status
uint8 status                 # status (see chap. 8 of operation instructions)

# Barcode data
uint8 barcode                # barcode data (0 indicates no barcode)

# Device status
uint16 dev_status            # device status (0x2018, value 0 = okay, see chap. 8 of operation instructions)

# Extended code
uint32 extended_code         # extended code (0x2021sub9, see chap. 8 of operation instructions)

# Error register
uint8 error                  # error register (0x1001, value 0 = okay, see chap. 8 of operation instructions)

# barcode center point [m] (OLS20 only)
float32 barcode_center_point # OLS20 only (0x2021subA), OLS10: always 0

# quality of lines
uint8 quality_of_lines       # OLS20 only (0x2021subB), OLS10: always 0

# Intensity of lines 1 to 3
uint8[] intensity_of_lines   # OLS20 only (0x2023sub1 to 0x2023sub3), OLS10: always 0

