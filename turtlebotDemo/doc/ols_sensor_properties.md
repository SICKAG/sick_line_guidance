# Properties of OLS line guidance sensor for sick_line_guidance_demo

The OLS sensor on the demo system is mounted 65 mm over ground. Both line distances (lcp, line center points) and line width measured by the sensor 
depend on its height over ground. LineSensorConfig configures a scaling between sensor measurement and physical world:

| parameter | default value | details |
| --- | --- | --- |
| line_sensor_scaling_dist | 180.0 / 133.0 | Scaling between physical distance to the line center and measured line center point, depending on mounted sensor height (measurement: lcp = 180 mm, physical: lcp = 133 mm) |
| line_sensor_scaling_width |  29.0 / 20.0 | Scaling between physical line width (20 mm) and measured line width (29 mm), depending on mounted sensor height (sensor mounted 100 mm over ground: scaling = 1, sensor mounted 65 mm over ground: scaling = 100/65 = 1.5) |


## Line distance scaling

Line center points measured with the demo system (OLS mounted 65 mm over ground):

| lcp measured | physical line distance | scaling |
| --- | --- | --- |
| 180 mm | 133 mm | 180/133 = 1.35 |

Line offset: The sensor is not completely centered. If the sensor is centered over the right side of the line,
a line center point with value 0 is detected.

## Line width scaling

Line width measured with the demo system (OLS mounted 65 mm over ground):

| angle | measured width | physical width | scaling |
| --- | --- | --- | --- |
| 0  degree |  29 mm | 20 mm | 29/29 = 1.45 |
| 45  degree |  43 mm | 28 mm | 43/28 = 1.52 |
| max. |  75 mm |  |  |

## Line switching at junctions

Lines detected by the sensor can switch at junctions depending on robots movement. When taking the left turnout, the left line will become the center line 
and the previous main line will become the right line.

The following line switches were observed under a test of the demo system:

Sensor moves from left to right at a junction (2 lines, lcp\[0\], lcp\[1\], lcp\[2\] measured in mm):

| lcp\[0\] | lcp\[1\] | lcp\[2\] | description |
| --- | --- | --- | --- |
| - | 88 | - | start, one line visible |
| - | 0 | 46 | switched to 2 lines, when sensor in the center of left line |
| -32 | 12 | - | next line switch  |

Sensor moves from right to left at a junction (2 lines, lcp\[0\], lcp\[1\], lcp\[2\] measured in mm):


## Line detection at barcodes

If a barcode is placed over the line, lines are detected in different ways:
1. There are no lines detected within the lower textzone (the area with human readable numbers).
2. Within the label zone (machine readable barcode label), a line with a width of round about the barcode is detected (line width >= 80 mm).
3. If the yaw angle between barcode and sensor is above 30 degrees, two or three lines are detected with small width (i.e. the markers are detected as lines).
4. No lines are detected within the small upper gap between the label area and the upper barcode border. 
