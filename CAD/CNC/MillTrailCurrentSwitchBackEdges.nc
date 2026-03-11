(Exported by FreeCAD)
(Post Processor: grbl_post)
(Output Time:2026-03-11 09:52:27.102363)
(Begin preamble)
G17 G90
G21
(Begin operation: Fixture)
(Path: Fixture)
G54
(Finish operation: Fixture)
(Begin operation: TC: SpeTool O Flute 1/4")
(Path: TC: SpeTool O Flute 1/4")
(TC: SpeTool O Flute 1/4")
(Begin toolchange)
( M6 T2 )
M3 S18000
(Finish operation: TC: SpeTool O Flute 1/4")
(Begin operation: Profile)
(Path: Profile)
(Profile)
(Compensated Tool Path. Diameter: 6.35)
G0 Z5.000
G0 X27.645 Y54.045
G0 Z3.000
G1 X27.645 Y54.045 Z-1.620 F75.000
G2 X28.575 Y51.800 Z-1.620 I-2.245 J-2.245 K0.000 F75.000
G1 X28.575 Y0.000 Z-1.620 F75.000
G2 X25.400 Y-3.175 Z-1.620 I-3.175 J0.000 K0.000 F75.000
G1 X0.000 Y-3.175 Z-1.620 F75.000
G2 X-3.175 Y0.000 Z-1.620 I0.000 J3.175 K0.000 F75.000
G1 X-3.175 Y51.800 Z-1.620 F75.000
G2 X0.000 Y54.975 Z-1.620 I3.175 J-0.000 K0.000 F75.000
G1 X25.400 Y54.975 Z-1.620 F75.000
G2 X27.645 Y54.045 Z-1.620 I-0.000 J-3.175 K0.000 F75.000
G0 Z5.000
G0 Z5.000
(Finish operation: Profile)
(Begin postamble)
M5
G17 G90
M2
