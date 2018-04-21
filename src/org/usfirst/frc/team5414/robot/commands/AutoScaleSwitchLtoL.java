package org.usfirst.frc.team5414.robot.commands;

import java.util.ArrayList;
import java.util.StringTokenizer;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;
//import jaci.pathfinder.Pathfinder;

/**
 * Goes from left side of starting area to the front of the left scale
 */
public class AutoScaleSwitchLtoL extends CommandGroup {

    public AutoScaleSwitchLtoL() {
    	ArrayList<Double> left = new ArrayList<>();
    	ArrayList<Double> right = new ArrayList<>();
    	String path = "0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 2 1 3 2 3 2 4 3 6 3 7 4 8 5 10 6 12 8 14 9 17 11 19 12 22 14 24 16 27 18 31 20 34 23 38 25 41 28 45 30 49 33 53 36 57 40 61 43 65 46 70 50 74 54 78 57 83 61 88 66 93 70 97 74 102 78 107 82 112 87 117 92 122 96 127 101 133 106 138 111 143 115 149 121 155 126 160 132 166 136 171 142 177 147 183 153 189 159 195 165 201 170 207 176 213 183 220 189 225 195 232 201 238 208 244 214 251 220 257 227 264 234 269 240 276 247 282 254 289 261 296 268 302 275 309 282 315 289 322 296 328 303 335 311 342 319 349 326 356 334 363 342 370 349 377 357 383 365 394 377 397 381 404 389 411 397 418 405 425 413 433 422 440 429 447 438 454 445 461 454 469 462 476 470 482 477 490 485 498 494 506 502 513 509 521 518 529 526 536 533 544 542 552 550 560 558 568 565 576 573 584 581 592 589 600 596 609 604 617 612 626 619 634 626 643 634 651 642 660 649 668 657 677 664 686 673 695 680 704 687 713 695 722 703 730 710 739 718 748 725 757 733 765 740 773 748 782 755 792 764 799 771 809 780 817 787 825 796 833 804 842 812 851 820 859 829 868 837 876 846 884 855 893 863 901 871 909 880 917 889 925 898 933 907 942 916 950 925 958 934 966 943 975 952 983 961 992 971 999 979 1007 988 1015 997 1024 1006 1032 1015 1040 1024 1048 1033 1057 1042 1065 1050 1074 1059 1082 1068 1091 1077 1100 1087 1111 1099 1116 1104 1124 1113 1132 1121 1141 1130 1149 1139 1158 1149 1167 1157 1175 1166 1184 1175 1192 1183 1201 1191 1209 1200 1217 1208 1226 1217 1234 1224 1242 1232 1250 1240 1257 1247 1265 1254 1273 1261 1280 1268 1287 1275 1295 1281 1301 1287 1309 1294 1315 1299 1323 1305 1329 1311 1336 1317 1343 1322 1349 1328 1355 1333 1362 1338 1368 1344 1374 1349 1381 1354 1387 1360 1393 1365 1399 1370 1405 1375 1411 1380 1417 1384 1422 1389 1427 1393 1433 1398 1437 1401 1443 1405 1448 1409 1452 1412 1457 1416";
    	int skip = 2;
    	int count = 0;
    	int startingKickSkip = 50;
    	StringTokenizer st = new StringTokenizer(path);
    	
    	try {	
    		for(int i = 0; i < startingKickSkip; i++) 
    		{
    			st.nextToken();
    			st.nextToken();
    		}
    		while(st.hasMoreTokens())
        	{
    			if(count++ == skip)
    			{
					double l = Double.parseDouble(st.nextToken());
					double r = Double.parseDouble(st.nextToken());
					left.add(l);
					right.add(r);
					count = 0;
    			}
    			else
    			{
    				try {
    				st.nextToken();
    				st.nextToken();
    				} catch(Exception e) {}
    			}
        	}
    	} catch(Exception e) {
    		e.printStackTrace();
    		Robot.drivetrain.stop();
    		DriverStation.reportWarning("NOOOO", true);
    	}
    	addSequential(new ZeroGyro());
    	addParallel(new ArmSetHover());
    	addSequential(new FollowEncoder(left, right));
    	addSequential(new SetAngle(-162));
    	addSequential(new SetAngle(-162));
    	addSequential(new DriveForward(-1.4, 9999, 4.5, 3.3));
    	addParallel(new SpintakeSet(-.3,-.3));
    	addSequential(new ArmThrowbackHigh());
    	addSequential(new SpintakeStop());
    	addParallel(new ArmSetLow());
    	addSequential(new SetAngle(170));
    	addSequential(new SetAngle(170));
    	addParallel(new SpintakeSet(1,1));
    	addSequential(new DriveForward(2,2));
    	addParallel(new SpintakePushIn());
    	addSequential(new DriveForward(-.45,9999, 4.5, 3.5));
    	addParallel(new SpintakeSet(-.6,.6));
    	addParallel(new ArmSetLow());
    	addSequential(new Wait(.4));
    	addParallel(new SpintakeSet(1,1));
    	addSequential(new DriveForward(.12));
    	addParallel(new SpintakeStop());
    	addParallel(new SpintakePushOut());
    	addParallel(new ArmPincherClose());
    	addSequential(new SetAngle(165));
    	addParallel(new ArmSetAngle(130));
    	addSequential(new DriveForward(.6, 1.));
    	addSequential(new ArmPincherOpen());
    }
}
