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
public class AutoScaleLtoL extends CommandGroup {

    public AutoScaleLtoL() {
    	ArrayList<Double> left = new ArrayList<>();
    	ArrayList<Double> right = new ArrayList<>();
    	String path = "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 2 2 2 2 3 3 4 4 4 4 5 5 6 6 7 7 9 8 10 10 11 11 13 13 14 14 16 16 18 18 19 20 21 22 23 24 25 26 27 28 29 30 32 32 34 35 36 37 39 40 41 42 44 45 46 48 50 52 52 54 55 56 58 59 61 62 63 65 66 68 69 71 73 75 76 78 79 81 82 84 85 87 89 91 92 94 96 98 99 101 103 104 106 108 110 111 113 115 117 118 120 122 124 126 128 129 131 133 135 137 139 141 143 144 146 148 150 152 154 155 158 159 162 163 166 166 169 170 173 174 177 178 181 181 185 185 189 189 194 193 198 197 202 200 206 204 210 208 214 212 218 215 222 219 226 223 230 227 235 231 239 235 243 239 247 243 251 247 255 251 259 254 264 258 268 262 272 266 276 270 280 274 285 279 289 283 293 287 298 291 302 295 306 299 311 304 315 308 319 312 324 316 328 321 333 325 337 330 342 334 347 339 352 344 357 348 362 353 367 358 371 363 378 370 381 373 386 378 392 383 397 388 403 394 408 399 414 405 419 411 425 417 431 422 437 428 443 434 449 440 455 446 461 452 468 459 474 465 480 471 487 478 493 484 500 491 506 497 512 504 519 510 526 517 532 524 539 530 546 537 553 544 559 551 566 558 574 566 580 572 587 579 594 586 601 593 608 600 615 607 622 614 629 621 636 628 643 635 650 642 658 649 665 656 672 663 679 671 686 678 694 685 701 692 708 699 716 707 723 714 730 722 738 729 745 736 752 743 760 751 767 758 774 766 782 773 789 781 796 788 804 796 811 803 818 811 826 818 833 825 841 833 848 840 855 848 863 855 871 863 878 870 885 878 892 885 900 893 907 900 915 908 922 915 929 922 936 930 950 943 952 945 959 952 966 960 974 968 981 975 989 983 996 990 1003 997 1011 1005 1018 1012 1026 1019 1033 1027 1041 1034 1048 1041 1055 1049 1062 1056 1069 1062 1076 1068 1082 1075 1089 1081 1095 1087 1102 1094 1108 1100 1115 1106 1121 1111 1127 1117 1133 1123 1139 1128 1144 1134 1150 1139 1155 1144 1160 1149 1166 1154 1171 1159 1175 1164 1180 1168 1185 1173 1190 1177 1194 1181 1198 1186 1203 1190 1207 1194 1211 1198 1215 1201 1219 1205 1223 1209 1227 1212 1231 1216 1235 1220 1238 1223 1242 1227 1246 1230 1249 1234 1252 1237 1256 1240 1259 1243 1262 1246 1265 1249 1268 1252 1271 1255 1274 1258 1277 1261 1280 1263 1283 1266 1286 1269 1288 1271 1291 1273 1294 1276 1296 1278 1299 1280 1301 1283 1303 1285 1306 1287 1308 1289 1310 1291 1312 1293 1315 1296 1317 1297 1319 1299 1321 1302 1323 1304 1325 1305 1328 1308 1329 1309 1331 1311 1333 1312 1334 1314 1336 1315 1338 1317 1339 1319 1341 1320 1343 1322 1344 1323 1346 1325 1348 1326 1349 1328 1351 1329 1352 1331 1354 1332 1355 1334 1356 1335 1358 1336 1359 1338 1360 1339 1362 1340 1363 1342 1364 1343 1366 1345 1367 1346 1368 1347 1369 1349 1370 1350 1372 1351 1373 1352 1374 1353 1375 1355 1376 1356 1377 1357 1378 1358 1379 1359 1380 1361 1381 1362 1383 1363 1384 1364 1384 1365 1385 1366 1386 1367 1387 1368 1388 1369 1389 1371 1390 1372 1391 1373 1392 1374 1393 1375 1394 1376 1395 1377 1396 1378 1397 1379 1398 1380 1399 1381 1399 1382 1400 1383 1401 1384 1402 1385 1403 1386 1404 1387 1404 1388 1405 1389 1406 1390 1407 1392 1408 1393 1408 1394 1409 1395 1410 1396 1411 1397 1412 1398 1413 1398 1414 1399 1415 1400 1416 1401 1417 1402 1418 1403 1418 1404 1419 1405 1420 1406 1425 1411 1425 1411 1425 1411 1426 1412 1426 1412 1426 1412 1426 1412 1427 1413 1428 1414 1429 1415 1429 1416 1430 1417 1431 1418 1432 1419 1433 1419 1434 1420 1435 1421 1435 1422 1436 1423 1437 1424 1438 1424 1439 1425 1440 1426 1440 1427 1441 1427 1441 1428 1441 1428 1442 1429 1442 1429 1443 1429 1443 1430 1443 1430 1443 1430 1443 1430 1443 1430 1443 1430 1443 1430 1443 1430 1443 1430 1443 1430 1443 1430 1443 1430 1443 1430 1443 1430 1443 1430 1443 1430";
    	int skip = 3;
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
    	addSequential(new DriveForward(-1.1));
    	addParallel(new SpintakeSet(-.3,-.3));
    	addSequential(new ArmThrowbackHigh());
    	addParallel(new SpintakeStop());
    	addParallel(new ArmSetLow());
    	addSequential(new SetAngle(175));
    	addSequential(new SetAngle(175));
    	addParallel(new SpintakeSet(1,1));
    	addParallel(new SpintakePushIn());
    	addSequential(new DriveForward(1.5,2));
    	addSequential(new DriveForward(-1.2));
    	addParallel(new SpintakeSet(1,1));
    	addParallel(new SpintakeStop());
    	addParallel(new SpintakePushOut());
    	addParallel(new ArmPincherClose());
    	addParallel(new SetAngle(-168));
    	addSequential(new Wait(.3));
    	addSequential(new ArmThrowbackHigh());
    	addSequential(new ArmSetLow());
    	/*
    	addParallel(new ArmPincherClose());
    	addSequential(new Wait(.2));
    	addParallel(new ZeroGyro());
    	addParallel(new ArmSetHover());
//    	addSequential(new DriveForward(3));
//    	addSequential(new SetAngle(0));
//    	addSequential(new DriveForward(3.1));
    	addSequential(new DriveForward(6.1));
    	addSequential(new TurnLeft(133));
    	addSequential(new DriveForward(-.5));
    	addSequential(new ArmThrowbackHigh());
    	addParallel(new ArmSetLow());
    	addSequential(new SetAngle(148));
    	addSequential(new DriveForward(1.35));
    	addSequential(new ArmPincherClose());
    	addSequential(new DriveForward(-.3));
    	addSequential(new SetAngle(-170));
    	addSequential(new DriveForward(-1.));
    	addSequential(new ArmThrowbackLow());
    	addParallel(new ArmSetLow());
    	*/
    }
}
