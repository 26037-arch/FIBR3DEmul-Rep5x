-- Generated from config/rep5x_ender3_v3_se.json; do not edit by hand.
return {
    schemaVersion=1,
    lc=0.0,
    lb=0.05467,
    root='Rep5x_Ender3V3SE',
    joints={X='Rep5x_X_joint',Y='Rep5x_Y_bed_joint',Z='Rep5x_Z_joint',C='Rep5x_C_joint',B='Rep5x_B_joint'},
    objects={frame='Rep5x_FixedFrame',bed='Rep5x_MovingBed',gantry='Rep5x_ZGantry',carriage='Rep5x_XCarriage',cLink='Rep5x_CLink',bLink='Rep5x_BLink',nozzle='Rep5x_Nozzle',tip='Rep5x_NozzleTip',drawBoard='DrawBoard'},
    signs={X=1,Y=-1,Z=1,C=1,B=1},
    limits={X={0.0,0.2},Y={-0.04,0.2},Z={0.0,0.1746},B={-135.0*math.pi/180,135.0*math.pi/180}},
}
