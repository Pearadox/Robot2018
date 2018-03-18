package org.usfirst.frc.team5414.robot.commands;

import java.util.ArrayList;
import java.util.StringTokenizer;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoSwitchMtoR extends CommandGroup {

    public AutoSwitchMtoR() {
    	StringTokenizer st = null;
    	ArrayList<Double> left = new ArrayList<>();
    	ArrayList<Double> right = new ArrayList<>();
    	String path = "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 1 0 1 1 2 1 2 1 3 1 3 2 4 2 5 2 6 3 7 4 8 4 10 5 11 5 12 6 14 7 15 8 17 9 18 10 20 10 21 11 23 12 25 13 27 15 28 16 30 17 32 18 34 20 36 21 38 22 40 24 43 25 45 26 47 28 49 29 52 31 54 33 56 34 59 36 61 37 63 39 66 41 68 43 71 44 73 46 76 48 79 49 81 51 84 53 87 55 89 56 92 58 95 60 98 62 100 64 103 66 106 68 109 69 112 71 115 73 118 75 120 77 123 79 126 81 129 83 132 85 135 86 138 88 141 90 144 92 147 94 150 96 153 98 156 100 159 102 162 104 165 106 168 108 171 110 174 112 177 114 180 116 183 118 186 121 188 123 191 125 194 127 197 129 200 132 203 134 206 136 208 138 211 141 214 143 217 145 220 148 223 150 225 153 228 155 230 158 232 160 235 163 237 166 240 168 242 171 244 174 247 177 249 180 251 183 253 186 255 189 257 193 259 196 261 199 263 202 265 206 267 209 268 212 271 215 272 218 274 222 276 225 279 229 281 231 283 234 285 237 288 240 290 242 292 245 295 248 297 251 300 254 302 257 305 259 308 262 310 265 313 267 316 270 319 272 322 275 325 277 328 280 331 282 334 285 337 287 340 290 343 292 347 295 350 297 353 300 356 302 359 304 362 307 365 309 368 312 372 314 375 317 378 319 381 321 384 324 387 326 390 329 393 331 396 334 398 336 401 339 404 341 407 344 410 347 412 349 415 352 417 355 420 358 423 361 425 364 428 366 430 369 432 372 435 375 437 378 439 382 441 385 444 388 446 391 448 394 450 398 452 401 454 404 457 407 459 410 461 413 463 416 465 419 467 422 470 426 472 429 474 432 477 434 479 437 481 440 483 443 486 446 488 449 491 452 493 455 496 458 499 461 501 463 504 466 506 469 509 472 511 475 514 477 516 480 519 483 522 486 524 488 527 491 529 494 532 497 534 499 537 502 539 505 541 508 544 511 546 514 549 517 551 520 553 523 555 526 557 529 560 532 562 535 564 538 566 541 568 544 570 547 572 551 575 554 577 557 579 560 581 563 583 566 585 568 588 571 590 574 592 577 595 580 597 583 599 586 602 589 604 592 606 594 609 597 611 600 614 603 616 605 619 608 621 611 624 613 626 616 629 619 631 621 634 624 636 627 638 629 641 632 643 635 646 638 648 640 651 643 653 646 656 649 658 652 660 654 662 657 665 660 667 663 668 666 670 668 671 671 671 674 672 676 673 678 674 679 674 679 675 678 675 677 675 675 675 674 675 674 675 673 675 672 675 672 675 672 676 672 676 672 676 672 676 672 676 672 676 673 676 673 676 673 676 673 676 673 676 673 676 673";
    	int skip = 1;
    	int count = 0;
    	int startingKickSkip = 50;
    	st = new StringTokenizer(path);
    	
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
    	
//    	/*
    	addParallel(new ZeroGyro());
    	addParallel(new ArmPincherClose());
    	addSequential(new Wait(.7));
    	addParallel(new ArmSetSwitch());
    	addSequential(new FollowEncoder(left, right));
    	addParallel(new DriveForward(.4));
    	addSequential(new Wait(1));
    	addSequential(new ArmPincherOpen());
    	addSequential(new Wait(.5));
    	addSequential(new DriveForward(-1));
    	addSequential(new SetAngle(90));
    	addSequential(new DriveForward(.7));
//		*/
//    	addSequential(new FollowEncoder(left, right));
    }
}
