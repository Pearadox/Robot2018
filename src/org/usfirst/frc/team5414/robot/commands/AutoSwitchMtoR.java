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
    	String path = "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 1 0 1 0 1 0 2 0 2 0 2 1 3 2 3 2 4 2 5 2 6 3 6 3 7 4 8 4 9 5 10 5 11 6 13 6 14 7 15 7 17 8 18 9 19 10 21 10 23 11 24 12 26 13 28 14 29 15 31 16 33 17 35 18 37 19 39 20 41 21 43 22 46 24 48 25 50 26 52 28 55 29 57 30 60 32 62 33 65 35 67 36 70 38 73 40 75 41 78 43 80 44 83 46 86 48 89 50 92 51 95 53 97 55 100 57 103 59 106 61 109 63 112 65 115 67 118 69 121 71 124 73 127 75 130 78 133 80 137 82 140 84 142 87 145 89 148 91 151 94 154 96 157 99 160 101 163 104 166 106 169 109 172 111 175 114 178 117 181 119 184 122 187 125 190 127 192 130 195 133 198 135 201 138 204 141 207 144 210 147 213 149 216 152 219 155 221 158 224 160 227 163 230 166 233 169 236 172 239 174 242 177 245 180 247 183 250 185 253 188 256 191 259 194 262 197 265 200 268 202 271 205 273 208 276 210 279 213 282 216 285 219 288 221 291 224 294 227 297 230 300 232 303 235 306 238 309 240 312 243 315 245 318 248 321 251 324 253 327 256 330 259 333 261 336 264 339 267 342 269 345 272 348 274 351 277 354 280 357 282 360 285 363 287 366 290 369 293 373 296 375 298 379 301 382 304 384 306 387 309 390 311 393 314 396 316 399 319 402 322 405 324 407 327 413 332 414 333 416 335 419 338 421 341 424 343 427 346 429 349 432 352 434 354 437 357 439 360 442 363 444 365 447 368 449 371 452 373 454 376 457 379 459 381 462 384 464 387 467 390 469 393 472 395 475 398 477 401 479 403 482 406 484 409 487 412 489 414 492 417 494 420 497 423 499 426 501 428 504 431 506 434 508 437 510 439 512 442 515 445 517 448 519 450 521 453 523 456 526 459 528 462 530 465 532 467 534 470 536 473 538 476 540 479 542 481 544 484 546 487 548 490 550 493 552 496 554 499 556 501 558 504 560 507 562 510 564 513 566 516 568 518 570 521 572 524 574 527 576 529 578 532 580 535 582 537 584 540 586 543 588 545 590 548 592 551 594 553 596 556 598 559 600 561 601 564 603 566 605 568 607 571 609 573 611 576 613 578 614 581 616 583 618 585 620 588 622 590 624 593 626 595 628 597 630 599 632 602 634 604 636 606 637 609 639 611 641 613 643 615 645 618 647 620 649 622 651 624 653 627 655 629 657 631 658 633 660 635 662 637 663 639 665 640 667 642 668 644 670 645 671 647 672 648 673 649 674 651 675 652 676 653 677 654 678 655 679 656 680 656 681 657 681 658 682 658 683 659 683 660 684 660 684 660 684 661 685 661 685 661 685 661 685 661 685 661 685 661 685 661";
    	int skip = 4;
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
    	addSequential(new Wait(.1));
    	addParallel(new ArmSetSwitch());
    	addSequential(new FollowEncoder(left, right));
    	addSequential(new DriveForward(.9, .7));
    	addSequential(new ArmPincherOpen());
    	addParallel(new SpintakeSet(-.4,-.4));
    	addSequential(new Wait(.2));
    	addParallel(new SpintakeStop());
    	addSequential(new DriveForward(-.3));
    	addParallel(new ArmSetLow());
    	addSequential(new SetAngle(-56));
    	addParallel(new SpintakeSet(1,1));
    	addParallel(new SpintakePushIn());
    	addSequential(new DriveForward(2, 2.2));
    	addSequential(new DriveForward(-.9-.2, 1.5, 3.5, 3.2));  
    	addParallel(new SpintakeSet(.5,-.5));
    	addSequential(new Wait(.45));
    	addSequential(new SpintakePushOut());
    	addParallel(new SpintakeSet(1,1));
    	addParallel(new SpintakePushIn());
    	addSequential(new DriveForward(.1));
    	addSequential(new SetAngle(0));
    	addParallel(new SpintakeStop());
    	addParallel(new SpintakePushOut());
    	addParallel(new ArmPincherClose());
    	addSequential(new DriveForward(-.3));
    	addParallel(new ArmSetAngle(105));
    	addSequential(new DriveForward(.9+.1, 1.3, 4.5, 3));
    	addSequential(new ArmPincherOpen());
//		*/
//    	addSequential(new FollowEncoder(left, right));
    }
}
