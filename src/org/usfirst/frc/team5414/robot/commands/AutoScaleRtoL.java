package org.usfirst.frc.team5414.robot.commands;

import java.util.ArrayList;
import java.util.StringTokenizer;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;
//import jaci.pathfinder.Pathfinder;

/**
 * Goes from right side of starting area to the front of the left scale
 * 
 * Approximate Time: 6 sec
 */
public class AutoScaleRtoL extends CommandGroup {

    public AutoScaleRtoL() {
    	
    	StringTokenizer st = null;
    	ArrayList<Double> left = new ArrayList<>();
    	ArrayList<Double> right = new ArrayList<>();
    	String path = "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 2 2 2 2 3 3 4 4 5 5 6 6 7 8 8 9 10 11 12 13 14 15 16 17 18 19 20 22 23 24 26 27 28 30 31 33 35 37 38 40 41 44 45 47 49 51 53 55 57 59 61 64 65 68 69 73 74 77 78 82 83 87 88 92 93 97 98 102 103 107 108 112 113 117 119 122 124 128 129 133 135 138 141 144 147 150 152 155 158 161 164 167 170 173 177 178 182 184 189 190 195 196 201 202 207 208 214 214 220 221 227 227 233 233 239 240 246 246 253 252 259 259 266 265 273 272 280 278 286 285 293 291 300 298 306 304 313 311 320 318 327 324 334 331 341 338 348 345 355 351 362 358 369 365 376 372 383 379 390 386 397 393 404 399 411 406 418 414 426 420 432 427 440 434 447 441 454 448 461 456 468 463 475 470 483 477 490 484 497 491 504 499 511 506 518 513 526 521 533 528 540 535 547 543 554 550 561 557 569 564 576 572 583 579 590 587 597 594 604 602 611 610 619 618 626 625 632 633 639 641 646 649 653 657 660 665 667 674 674 681 680 690 687 698 694 706 701 715 708 723 714 732 721 740 727 749 734 757 740 765 747 772 753 780 759 788 766 795 772 802 778 810 784 817 790 824 796 831 802 838 808 844 813 851 819 857 824 864 830 870 835 876 841 882 846 888 851 894 856 899 861 905 866 910 871 915 876 921 881 926 886 931 890 937 895 942 900 946 905 951 909 956 914 961 918 966 923 970 928 975 932 980 937 985 942 989 946 994 950 999 955 1003 960 1008 964 1013 969 1017 973 1022 978 1026 982 1031 987 1035 991 1040 996 1044 1000 1049 1005 1053 1009 1058 1014 1062 1018 1067 1023 1071 1027 1075 1031 1080 1036 1084 1040 1089 1045 1093 1049 1098 1053 1102 1057 1107 1062 1111 1066 1116 1070 1120 1074 1125 1078 1130 1083 1134 1086 1139 1090 1143 1094 1148 1098 1153 1102 1158 1107 1163 1111 1167 1114 1172 1118 1177 1122 1182 1126 1186 1130 1191 1134 1196 1138 1201 1141 1206 1145 1211 1149 1216 1152 1221 1156 1226 1160 1231 1163 1236 1167 1241 1171 1247 1174 1252 1178 1257 1181 1262 1185 1267 1188 1273 1191 1278 1195 1283 1198 1288 1201 1294 1205 1299 1208 1304 1211 1310 1214 1315 1218 1320 1221 1325 1224 1331 1227 1336 1230 1342 1233 1347 1237 1352 1240 1358 1243 1363 1246 1368 1249 1374 1252 1379 1255 1385 1258 1390 1261 1396 1264 1401 1267 1407 1270 1412 1273 1418 1276 1423 1279 1429 1283 1433 1286 1439 1289 1444 1292 1449 1295 1455 1299 1459 1302 1464 1305 1469 1308 1474 1312 1479 1316 1483 1319 1488 1322 1493 1326 1498 1330 1503 1333 1507 1337 1512 1341 1516 1345 1521 1349 1525 1352 1530 1356 1535 1360 1539 1364 1543 1368 1548 1372 1552 1375 1556 1379 1561 1383 1565 1388 1569 1392 1573 1396 1578 1400 1582 1404 1586 1408 1590 1412 1595 1416 1599 1420 1603 1424 1607 1428 1612 1433 1616 1437 1620 1441 1624 1445 1628 1449 1632 1453 1637 1458 1640 1461 1644 1465 1649 1469 1653 1473 1657 1478 1661 1482 1666 1486 1670 1490 1674 1494 1678 1498 1682 1502 1687 1505 1691 1509 1696 1513 1700 1517 1705 1521 1709 1525 1713 1529 1718 1532 1723 1536 1727 1540 1732 1543 1737 1547 1742 1551 1747 1554 1752 1558 1756 1561 1762 1564 1767 1568 1772 1571 1777 1574 1784 1578 1791 1582 1797 1586 1801 1590 1806 1593 1810 1597 1815 1601 1820 1605 1824 1608 1828 1611 1833 1615 1837 1618 1842 1622 1846 1625 1850 1628 1855 1631 1860 1634 1864 1637 1868 1640 1873 1643 1877 1646 1882 1649 1888 1652 1892 1653 1897 1655 1902 1657 1907 1659 1912 1660 1917 1661 1922 1663 1927 1664 1932 1666 1937 1667 1942 1669 1948 1670 1953 1672 1958 1674 1963 1676 1968 1677 1974 1679 1979 1681 1983 1683 1988 1685 1992 1686 1997 1688 2001 1689 2005 1691 2009 1693 2013 1695 2017 1696 2021 1699 2025 1701 2028 1703 2031 1705 2033 1708 2036 1710 2038 1713 2041 1715 2043 1718 2045 1720 2047 1723 2049 1725 2050 1727 2052 1730 2054 1732 2056 1735 2057 1737 2058 1739 2059 1741 2060 1743 2062 1745 2063 1747 2064 1749 2066 1751 2067 1753 2067 1755 2068 1756 2069 1758 2069 1760 2070 1761 2071 1763 2072 1765 2073 1766 2073 1768 2074 1769 2074 1771 2075 1772 2076 1774 2076 1776 2077 1777 2078 1778 2079 1780 2080 1781 2081 1782 2082 1784 2083 1785 2084 1787 2085 1788 2086 1789 2087 1791 2089 1792 2090 1794 2091 1795 2093 1797 2094 1799 2095 1800 2097 1802 2099 1804 2100 1806 2102 1807 2104 1809 2106 1811 2108 1813 2110 1815 2112 1817 2114 1819 2116 1821 2118 1823 2121 1825 2123 1828 2126 1830 2128 1832 2131 1835 2134 1837 2136 1839 2139 1842 2142 1845 2145 1847 2148 1850 2151 1853 2154 1856 2157 1859 2161 1862 2164 1865 2167 1868 2171 1872 2174 1875 2178 1878 2182 1882 2185 1885 2189 1889 2193 1892 2196 1896 2200 1900 2204 1904 2208 1908 2212 1912 2216 1917 2221 1922 2225 1926 2230 1929 2233 1933 2237 1938 2241 1943 2245 1947 2249 1951 2253 1956 2257 1960 2261 1964 2265 1969 2270 1973 2274 1978 2278 1982 2283 1987 2287 1991 2292 1995 2297 1998 2301 2003 2306 2007 2309 2011 2313 2014 2317 2018 2321 2023 2325 2027 2329 2032 2333 2037 2337 2042 2342 2047 2345 2052 2350 2056 2354 2061 2358 2065 2363 2070 2367 2075 2371 2080 2376 2083 2381 2087 2386 2090 2391 2094 2396 2098 2401 2102 2406 2107 2411 2111 2416 2115 2421 2120 2426 2124 2431 2128 2435 2132 2440 2137 2445 2141 2450 2146 2455 2151 2459 2155 2464 2160 2469 2165 2473 2169 2478 2174 2483 2179 2487 2184 2492 2189 2497 2194 2502 2199 2507 2204 2511 2209 2516 2214 2521 2219 2525 2224 2530 2229 2535 2234 2540 2240 2544 2245 2549 2250 2554 2255 2558 2260 2563 2265 2568 2270 2573 2276 2577 2281 2582 2286 2587 2291 2592 2296 2597 2302 2602 2307 2607 2312 2611 2317 2616 2322 2621 2327 2626 2332 2631 2337 2636 2343 2641 2348 2646 2353 2651 2358 2656 2363 2660 2368 2666 2373 2671 2378 2675 2383 2680 2387 2685 2392 2689 2397 2694 2402 2698 2406 2703 2411 2707 2415 2712 2420 2716 2425 2721 2429 2725 2433 2730 2438 2734 2442 2738 2446 2743 2450 2747 2455 2751 2460 2755 2464 2759 2469 2764 2474 2768 2479 2771 2484 2775 2489 2778 2494 2782 2499 2785 2504 2789 2509 2792 2513 2795 2518 2798 2523 2801 2528 2804 2533 2807 2537 2810 2542 2813 2547 2815 2551 2818 2555 2820 2560 2823 2564 2825 2568 2828 2572 2830 2576 2833 2580 2836 2584 2838 2588 2840 2592 2843 2596 2845 2599 2847 2603 2850 2606 2852 2610 2855 2614 2857 2617 2859 2620 2861 2624 2863 2627 2866 2630 2868 2633 2870 2636 2872 2640 2874 2643 2876 2645 2878 2648 2880 2651 2882 2654 2884 2657 2886 2660 2889 2663 2890 2665 2892 2668 2894 2671 2896 2674 2898 2676 2900 2679 2902 2682 2904 2684 2905 2687 2907 2689 2909 2692 2911 2694 2913 2697 2914 2699 2918 2704 2919 2705 2920 2706 2921 2708 2923 2711 2925 2713 2927 2715 2928 2717 2930 2720 2932 2722 2934 2724 2935 2726 2937 2728 2939 2730 2940 2732 2942 2735 2944 2737 2945 2739 2947 2741 2948 2743 2950 2745 2951 2747 2953 2749 2955 2751 2956 2753 2958 2755 2959 2757 2961 2759 2962 2761 2964 2763 2965 2765 2967 2767 2968 2769 2970 2771 2971 2773 2972 2775 2974 2777 2975 2779 2976 2781 2978 2784 2979 2786 2980 2788 2982 2790 2983 2792 2984 2794 2985 2796 2987 2798 2988 2800 2989 2802 2990 2805 2991 2807 2993 2809 2994 2811 2995 2813 2996 2815 2997 2818 2998 2820 2999 2822 3000 2825 3001 2827 3002 2830 3003 2832 3004 2835 3005 2837 3006 2839 3007 2842 3007 2844 3008 2847 3009 2850 3010 2852 3011 2855 3011 2858 3012 2860 3013 2863 3013 2866 3014 2869 3015 2872 3015 2874 3016 2877 3016 2880 3017 2883 3017 2886 3017 2889 3018 2892 3018 2895 3019 2898 3019 2901 3019 2904 3019 2908 3020 2911 3020 2914 3020 2917 3021 2920 3021 2923 3022 2925 3022 2928 3023 2931 3023 2934 3024 2937 3025 2939 3026 2942 3027 2945 3028 2948 3029 2950 3031 2953 3032 2956 3034 2959 3036 2962 3038 2965 3040 2968 3042 2972 3045 2975 3048 2978 3051 2982 3054 2986 3057 2990 3061 2995 3065 2999 3070 3004 3074 3009 3080 3015 3084 3020 3090 3026 3096 3033 3102 3040 3108 3046 3115 3053 3121 3060 3128 3067 3135 3075 3143 3083 3150 3091 3158 3099 3166 3107 3174 3115 3182 3124 3191 3133 3200 3142 3208 3151 3217 3160 3226 3169 3235 3179 3244 3188 3254 3198 3266 3210 3276 3222 3286 3233 3295 3244 3305 3254 3314 3264 3324 3274 3333 3284 3343 3293 3354 3304 3366 3315 3378 3328 3390 3339 3398 3350 3405 3357 3412 3364 3418 3370 3424 3374 3429 3378 3433 3381 3439";
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
    	addParallel(new ArmPincherClose());
		addParallel(new ZeroGyro());
		addSequential(new Wait(.3));
		addParallel(new ArmSetAngle(60));
		addSequential(new TurnRight(4.5));
    	addSequential(new FollowEncoder(left, right));
    	addSequential(new SetAngle(0));
		addSequential(new DriveForward(1.4));
    	addSequential(new SetAngle(-90));
    	addSequential(new ArmThrowbackHigh());
    	addParallel(new ArmSetSwitch());
    }
}
