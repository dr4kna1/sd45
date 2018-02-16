/* 
 * File:   tables.h
 * Author: rdotd
 *
 * 
 */

#ifndef TABLES_H
#define	TABLES_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* TABLES_H */

#define PWM_capacity 8              /* PWM bit range accuracy */

/* 71 meaningfull PWM period points which correspond actual flow on pump, 
 gathered by in-field measurment;
 period dimensioned in tmr3 ticks */
unsigned long per_pwm_arr[71] = {
2840237,
2603550,
2366864,//2028740,
2028740,//1775148,
1775148,//1577909,
1577909,//1775148,//1420118,        // 1.0 litr
1434463,//1577909,//1291017,        // 1.1
1237224,//1291017,//1183432,                  // 1.2
1092399,                            // 1.3
1237224,//1014370,                  // 1.4
1092399,//1014370,//946746 ,
1014370,//946746,//887574 ,
946746,//887574,//835364 ,
887574,//835364,//788955 ,
861469,//835364,//788955,//747431 ,
788955,//747431,//710059 ,
747431,//710059,//676247 ,
710059,//676247,//645508 ,
676247,//617443 ,
645508,//591716 ,
617443,//568047 ,
591716,//546199 ,
546199,//568047,//546199,//525970 ,
525970,//525970,//507185 ,
507185,//507185,//489696 ,
489696 ,//473373 ,
458103 ,
443787 ,
430339 ,
417682 ,
405748 ,
394477 ,
383816 ,
373715 ,
364133 ,
355030 ,
346370 ,
338123 ,
330260 ,
322754 ,
315582 ,
308721 ,
302153 ,
295858 ,
289820 ,
284024 ,
278455 ,
273100 ,
267947 ,
262985 ,
258203 ,
253593 ,
249144 ,
244848 ,
240698 ,
236686 ,
232806 ,
229051 ,
225416 ,
221893 ,
218480 ,
215169 ,
211958 ,
208841 ,
205814 ,
202874 ,
200017 ,
197239 ,
194537 ,
191908 ,
189349 
};

 unsigned long *per_pwm_p = per_pwm_arr;
 
#if PWM_capacity == 8
 /* Duty cycle of PWM */
 unsigned int pwm_arr[71] = {
229,
227,
224,
221,
218,
199,//216,              // 1.0 litr
183,//213,
180,//210,              // 1.2
174,//87,////207,              // 1.3
216,//85,                    // 1.4 
205,// 202
202,//199,
199,//196,
194,//194,
196,//191,
188,
185,
183,
185,//180,
184,//  177,
180,//174,
177,//172,
169,
169,//166,
163,
161,
158,
155,
152,
150,
147,
144,
141,
139,
136,
133,
130,
127,
125,
122,
119,
117,
114,
111,
108,  
106,
103,
100,
97 ,
94 ,
92 ,
89 ,
86 ,
83 ,
81 ,
78 ,
75 ,
72 ,
70 ,
67 ,
64 ,
62 ,
59 ,
56 ,
53 ,
51 ,
48 ,
45 ,
43 ,
39 ,
36
};
#elif PWM_capacity == 10
 unsigned int pwm_arr[71] = {
 916,
908,
896,
884,
872,
864,
852,
840,
828,
820,
820,
816,
796,
776,
784,
752,
740,
732,
740,
736,
720,
708,
676,
676,
652,
644,
632,
620,
608,
600,
588,
576,
564,
556,
544,
532,
520,
508,
500,
488,
476,
468,
456,
444,
432,
424,
412,
400,
388,
376,
368,
356,
344,
332,
324,
312,
300,
288,
280,
268,
256,
248,
236,
224,
212,
204,
192,
180,
172,
156,
144
 };
#endif
 
 unsigned int *pwm_p = pwm_arr;
 
/* Table for chosen flow rate decimal digits
 *  [0xHHH = d.dd liters per minute], */ 
const   unsigned int norm_arr[71] = {
0x50 ,
0x60 ,
0x70 ,
0x80 ,
0x90 ,
0x100,
0x110,
0x120,
0x130,
0x140,
0x150,
0x160,
0x170,
0x180,
0x190,
0x200,
0x210,
0x220,
0x230,
0x240,
0x250,
0x260,
0x270,
0x280,
0x290,
0x300,
0x310,
0x320,
0x330,
0x340,
0x350,
0x360,
0x370,
0x380,
0x390,
0x400,
0x410,
0x420,
0x430,
0x440,
0x450,
0x460,
0x470,
0x480,
0x490,
0x500,
0x510,
0x520,
0x530,
0x540,
0x550,
0x560,
0x570,
0x580,
0x590,
0x600,
0x610,
0x620,
0x630,
0x640,
0x650,
0x660,
0x670,
0x680,
0x690,
0x700,
0x710,
0x720,
0x730,
0x740,
0x750
};

/* Table for feedback flow rate decimal digits:
* [0xHHH = d.dd liters per minute],
 0xAAA stands for NULL rate symbol "---" */
const  unsigned int rate_arr[768] = {
0x817,
0x816,
0x815,
0x814,
0x813,
0x812,
0x811,
0x810,
0x809,
0x808,
0x807,
0x806,
0x805,
0x804,
0x803,
0x802,
0x801,
0x800,
0x799,
0x798,
0x797,
0x796,
0x795,
0x794,
0x793,
0x792,
0x791,
0x790,
0x789,
0x788,
0x787,
0x786,
0x785,
0x784,
0x783,
0x782,
0x781,
0x780,
0x779,
0x778,
0x777,
0x776,
0x775,
0x774,
0x773,
0x772,
0x771,
0x770,
0x769,
0x768,
0x767,
0x766,
0x765,
0x764,
0x763,
0x762,
0x761,
0x760,
0x759,
0x758,
0x757,
0x756,
0x755,
0x754,
0x753,
0x752,
0x751,
0x750,
0x749,
0x748,
0x747,
0x746,
0x745,
0x744,
0x743,
0x742,
0x741,
0x740,
0x739,
0x738,
0x737,
0x736,
0x735,
0x734,
0x733,
0x732,
0x731,
0x730,
0x729,
0x728,
0x727,
0x726,
0x725,
0x724,
0x723,
0x722,
0x721,
0x720,
0x719,
0x718,
0x717,
0x716,
0x715,
0x714,
0x713,
0x712,
0x711,
0x710,
0x709,
0x708,
0x707,
0x706,
0x705,
0x704,
0x703,
0x702,
0x701,
0x700,
0x699,
0x698,
0x697,
0x696,
0x695,
0x694,
0x693,
0x692,
0x691,
0x690,
0x689,
0x688,
0x687,
0x686,
0x685,
0x684,
0x683,
0x682,
0x681,
0x680,
0x679,
0x678,
0x677,
0x676,
0x675,
0x674,
0x673,
0x672,
0x671,
0x670,
0x669,
0x668,
0x667,
0x666,
0x665,
0x664,
0x663,
0x662,
0x661,
0x660,
0x659,
0x658,
0x657,
0x656,
0x655,
0x654,
0x653,
0x652,
0x651,
0x650,
0x649,
0x648,
0x647,
0x646,
0x645,
0x644,
0x643,
0x642,
0x641,
0x640,
0x639,
0x638,
0x637,
0x636,
0x635,
0x634,
0x633,
0x632,
0x631,
0x630,
0x629,
0x628,
0x627,
0x626,
0x625,
0x624,
0x623,
0x622,
0x621,
0x620,
0x619,
0x618,
0x617,
0x616,
0x615,
0x614,
0x613,
0x612,
0x611,
0x610,
0x609,
0x608,
0x607,
0x606,
0x605,
0x604,
0x603,
0x602,
0x601,
0x600,
0x599,
0x598,
0x597,
0x596,
0x595,
0x594,
0x593,
0x592,
0x591,
0x590,
0x589,
0x588,
0x587,
0x586,
0x585,
0x584,
0x583,
0x582,
0x581,
0x580,
0x579,
0x578,
0x577,
0x576,
0x575,
0x574,
0x573,
0x572,
0x571,
0x570,
0x569,
0x568,
0x567,
0x566,
0x565,
0x564,
0x563,
0x562,
0x561,
0x560,
0x559,
0x558,
0x557,
0x556,
0x555,
0x554,
0x553,
0x552,
0x551,
0x550,
0x549,
0x548,
0x547,
0x546,
0x545,
0x544,
0x543,
0x542,
0x541,
0x540,
0x539,
0x538,
0x537,
0x536,
0x535,
0x534,
0x533,
0x532,
0x531,
0x530,
0x529,
0x528,
0x527,
0x526,
0x525,
0x524,
0x523,
0x522,
0x521,
0x520,
0x519,
0x518,
0x517,
0x516,
0x515,
0x514,
0x513,
0x512,
0x511,
0x510,
0x509,
0x508,
0x507,
0x506,
0x505,
0x504,
0x503,
0x502,
0x501,
0x500,
0x499,
0x498,
0x497,
0x496,
0x495,
0x494,
0x493,
0x492,
0x491,
0x490,
0x489,
0x488,
0x487,
0x486,
0x485,
0x484,
0x483,
0x482,
0x481,
0x480,
0x479,
0x478,
0x477,
0x476,
0x475,
0x474,
0x473,
0x472,
0x471,
0x470,
0x469,
0x468,
0x467,
0x466,
0x465,
0x464,
0x463,
0x462,
0x461,
0x460,
0x459,
0x458,
0x457,
0x456,
0x455,
0x454,
0x453,
0x452,
0x451,
0x450,
0x449,
0x448,
0x447,
0x446,
0x445,
0x444,
0x443,
0x442,
0x441,
0x440,
0x439,
0x438,
0x437,
0x436,
0x435,
0x434,
0x433,
0x432,
0x431,
0x430,
0x429,
0x428,
0x427,
0x426,
0x425,
0x424,
0x423,
0x422,
0x421,
0x420,
0x419,
0x418,
0x417,
0x416,
0x415,
0x414,
0x413,
0x412,
0x411,
0x410,
0x409,
0x408,
0x407,
0x406,
0x405,
0x404,
0x403,
0x402,
0x401,
0x400,
0x399,
0x398,
0x397,
0x396,
0x395,
0x394,
0x393,
0x392,
0x391,
0x390,
0x389,
0x388,
0x387,
0x386,
0x385,
0x384,
0x383,
0x382,
0x381,
0x380,
0x379,
0x378,
0x377,
0x376,
0x375,
0x374,
0x373,
0x372,
0x371,
0x370,
0x369,
0x368,
0x367,
0x366,
0x365,
0x364,
0x363,
0x362,
0x361,
0x360,
0x359,
0x358,
0x357,
0x356,
0x355,
0x354,
0x353,
0x352,
0x351,
0x350,
0x349,
0x348,
0x347,
0x346,
0x345,
0x344,
0x343,
0x342,
0x341,
0x340,
0x339,
0x338,
0x337,
0x336,
0x335,
0x334,
0x333,
0x332,
0x331,
0x330,
0x329,
0x328,
0x327,
0x326,
0x325,
0x324,
0x323,
0x322,
0x321,
0x320,
0x319,
0x318,
0x317,
0x316,
0x315,
0x314,
0x313,
0x312,
0x311,
0x310,
0x309,
0x308,
0x307,
0x306,
0x305,
0x304,
0x303,
0x302,
0x301,
0x300,
0x299,
0x298,
0x297,
0x296,
0x295,
0x294,
0x293,
0x292,
0x291,
0x290,
0x289,
0x288,
0x287,
0x286,
0x285,
0x284,
0x283,
0x282,
0x281,
0x280,
0x279,
0x278,
0x277,
0x276,
0x275,
0x274,
0x273,
0x272,
0x271,
0x270,
0x269,
0x268,
0x267,
0x266,
0x265,
0x264,
0x263,
0x262,
0x261,
0x260,
0x259,
0x258,
0x257,
0x256,
0x255,
0x254,
0x253,
0x252,
0x251,
0x250,
0x249,
0x248,
0x247,
0x246,
0x245,
0x244,
0x243,
0x242,
0x241,
0x240,
0x239,
0x238,
0x237,
0x236,
0x235,
0x234,
0x233,
0x232,
0x231,
0x230,
0x229,
0x228,
0x227,
0x226,
0x225,
0x224,
0x223,
0x222,
0x221,
0x220,
0x219,
0x218,
0x217,
0x216,
0x215,
0x214,
0x213,
0x212,
0x211,
0x210,
0x209,
0x208,
0x207,
0x206,
0x205,
0x204,
0x203,
0x202,
0x201,
0x200,
0x199,
0x198,
0x197,
0x196,
0x195,
0x194,
0x193,
0x192,
0x191,
0x190,
0x189,
0x188,
0x187,
0x186,
0x185,
0x184,
0x183,
0x182,
0x181,
0x180,
0x179,
0x178,
0x177,
0x176,
0x175,
0x174,
0x173,
0x172,
0x171,
0x170,
0x169,
0x168,
0x167,
0x166,
0x165,
0x164,
0x163,
0x162,
0x161,
0x160,
0x159,
0x158,
0x157,
0x156,
0x155,
0x154,
0x153,
0x152,
0x151,
0x150,
0x149,
0x148,
0x147,
0x146,
0x145,
0x144,
0x143,
0x142,
0x141,
0x140,
0x139,
0x138,
0x137,
0x136,
0x135,
0x134,
0x133,
0x132,
0x131,
0x130,
0x129,
0x128,
0x127,
0x126,
0x125,
0x124,
0x123,
0x122,
0x121,
0x120,
0x119,
0x118,
0x117,
0x116,
0x115,
0x114,
0x113,
0x112,
0x111,
0x110,
0x109,
0x108,
0x107,
0x106,
0x105,
0x104,
0x103,
0x102,
0x101,
0x100,
0x99,
0x98,
0x97,
0x96,
0x95,
0x94,
0x93,
0x92,
0x91,
0x90,
0x89,
0x88,
0x87,
0x86,
0x85,
0x84,
0x83,
0x82,
0x81,
0x80,
0x79,
0x78,
0x77,
0x76,
0x75,
0x74,
0x73,
0x72,
0x71,
0x70,
0x69,
0x68,
0x67,
0x66,
0x65,
0x64,
0x63,
0x62,
0x61,
0x60,
0x59,
0x58,
0x57,
0x56,
0x55,
0x54,
0x53,
0x52,
0x51,
0xAAA
} ;

/* table for searching measured flow rate position in rate_arr table */
const   unsigned long meash_arr[768] ={
173821  ,
174034	,
174248	,
174462	,
174676	,
174891  ,
175107  ,
175323	,
175540	,
175757	,
175975  ,
176193  ,
176412  ,
176632	,
176852  ,
177072  ,
177293  ,
177515  ,
177737  ,
177960  ,
178183  ,
178407  ,
178631  ,
178856  ,
179082  ,
179308	,
179535  ,
179762  ,
179990  ,
180218  ,
180447  ,
180677  ,
180907  ,
181138  ,
181369  ,
181601  ,
181833  ,
182066  ,
182300  ,
182534  ,
182769  ,
183005  ,
183241  ,
183478  ,
183715  ,
183953  ,
184192  ,
184431  ,
184671  ,
184911	,
185152  ,
185394  ,
185636  ,
185879  ,
186123  ,
186367  ,
186612  ,
186858  ,
187104  ,
187351  ,
187598  ,
187846  ,
188095  ,
188345  ,
188595  ,
188846  ,
189097  ,
189349  ,
189602  ,
189855  ,
190110  ,
190364  ,
190620  ,
190876  ,
191133  ,
191391  ,
191649  ,
191908  ,
192168  ,
192428  ,
192689  ,
192951  ,
193213  ,
193477  ,
193741  ,
194005  ,
194271  ,
194537  ,
194804  ,
195071  ,
195340  ,
195609  ,
195878  ,
196149  ,
196420  ,
196692  ,
196965  ,
197239	,
197513  ,
197788  ,
198064  ,
198341  ,
198618  ,
198896  ,
199175  ,
199455  ,
199735  ,
200017  ,
200299  ,
200582  ,
200865  ,
201150  ,
201435  ,
201721  ,
202008  ,
202296  ,
202585  ,
202874  ,
203164  ,
203455  ,
203747  ,
204040  ,
204334  ,
204628  ,
204923  ,
205219  ,
205516  ,
205814  ,
206113  ,
206413  ,
206713  ,
207014  ,
207317  ,
207620  ,
207924  ,
208228  ,
208534  ,
208841  ,
209149  ,
209457  ,
209766  ,
210077  ,
210388  ,
210700  ,
211013  ,
211327  ,
211642  ,
211958  ,
212275  ,
212593  ,
212911  ,
213231  ,
213552  ,
213873  ,
214196  ,
214519  ,
214844  ,
215169  ,
215496  ,
215823  ,
216152  ,
216481  ,
216812  ,
217143  ,
217476  ,
217810  ,
218144  ,
218480  ,
218816  ,
219154  ,
219493  ,
219833  ,
220173  ,
220515  ,
220858  ,
221202  ,
221547  ,
221893  ,
222241  ,
222589  ,
222939  ,
223289  ,
223641  ,
223993  ,
224347  ,
224702  ,
225058  ,
225416  ,
225774  ,
226133  ,
226494  ,
226856  ,
227219  ,
227583	,
227948  ,
228315  ,
228683  ,
229051  ,
229421  ,
229793  ,
230165  ,
230539  ,
230914  ,
231290  ,
231667  ,
232045  ,
232425  ,
232806  ,
233189  ,
233572  ,
233957  ,
234343  ,
234730  ,
235119  ,
235509  ,
235900  ,
236293  ,
236686  ,
237082  ,
237478  ,
237876  ,
238275  ,
238675  ,
239077  ,
239480  ,
239885  ,
240291  ,
240698  ,
241107  ,
241517  ,
241928  ,
242341  ,
242755  ,
243171  ,
243588  ,
244007  ,
244427  ,
244848  ,
245271  ,
245695  ,
246121  ,
246548  ,
246977  ,
247407  ,
247839  ,
248272  ,
248707  ,
249144  ,
249581  ,
250021  ,
250462  ,
250904  ,
251348  ,
251794  ,
252241  ,
252690  ,
253141  ,
253593  ,
254046  ,
254501  ,
254958  ,
255417  ,
255877  ,
256339  ,
256803  ,
257268  ,
257735  ,
258203  ,
258674  ,
259146  ,
259619  ,
260095  ,
260572  ,
261051  ,
261532  ,
262014  ,
262499  ,
262985  ,
263473  ,
263963  ,
264454  ,
264947  ,
265443  ,
265940  ,
266439  ,
266940  ,
267442  ,
267947  ,
268453  ,
268962  ,
269472  ,
269984  ,
270499  ,
271015  ,
271533  ,
272053  ,
272575  ,
273100  ,
273626  ,
274154  ,
274684  ,
275217  ,
275751  ,
276288  ,
276826  ,
277367  ,
277910  ,
278455  ,
279002  ,
279551  ,
280102  ,
280656  ,
281212  ,
281770  ,
282330  ,
282892  ,
283457  ,
284024  ,
284593  ,
285164  ,
285738  ,
286314  ,
286893  ,
287473  ,
288056  ,
288642  ,
289230  ,
289820  ,
290413  ,
291008  ,
291605  ,
292205  ,
292808  ,
293413  ,
294020  ,
294630  ,
295243  ,
295858  ,
296476  ,
297096  ,
297719  ,
298344  ,
298972  ,
299603  ,
300236  ,
300873  ,
301511  ,
302153  ,
302797  ,
303444  ,
304094  ,
304746  ,
305402  ,
306060  ,
306721  ,
307385  ,
308052  ,
308721  ,
309394  ,
310070  ,
310748  ,
311429  ,
312114  ,
312801  ,
313492  ,
314185  ,
314882  ,
315582  ,
316285  ,
316991  ,
317700  ,
318412  ,
319128  ,
319846  ,
320568  ,
321294  ,
322022  ,
322754  ,
323489  ,
324228  ,
324970  ,
325715  ,
326464  ,
327216  ,
327972  ,
328731	,
329494  ,
330260  ,
331030  ,
331803  ,
332580  ,
333361  ,
334145  ,
334934  ,
335725  ,
336521  ,
337320  ,
338123  ,
338930  ,
339741  ,
340556  ,
341375  ,
342197  ,
343024  ,
343854  ,
344689  ,
345528  ,
346370  ,
347217  ,
348068  ,
348923  ,
349783  ,
350647  ,
351514  ,
352387  ,
353263  ,
354144  ,
355030  ,
355919  ,
356814  ,
357712  ,
358616  ,
359524  ,
360436  ,
361353  ,
362275  ,
363202  ,
364133  ,
365069  ,
366010  ,
366956  ,
367906  ,
368862  ,
369822  ,
370788  ,
371759  ,
372734  ,
373715  ,
374701  ,
375693  ,
376689  ,
377691  ,
378698  ,
379711  ,
380729  ,
381752  ,
382781  ,
383816  ,
384856  ,
385902  ,
386953  ,
388010  ,
389074  ,
390142  ,
391217  ,
392298  ,
393385  ,
394477  ,
395576  ,
396681  ,
397792  ,
398910  ,
400033  ,
401163  ,
402300  ,
403443  ,
404592  ,
405748  ,
406911  ,
408080  ,
409256  ,
410439  ,
411629  ,
412825  ,
414029  ,
415239  ,
416457  ,
417682  ,
418914  ,
420153  ,
421400  ,
422654  ,
423916  ,
425185  ,
426462  ,
427746  ,
429039  ,
430339  ,
431647  ,
432963  ,
434287  ,
435619  ,
436959  ,
438308  ,
439665  ,
441031  ,
442404  ,
443787  ,
445178  ,
446578  ,
447987  ,
449405  ,
450831  ,
452267  ,
453712  ,
455166  ,
456630  ,
458103  ,
459585  ,
461077  ,
462579  ,
464091  ,
465613  ,
467144  ,
468686  ,
470238  ,
471800  ,
473373  ,
474956  ,
476550  ,
478154  ,
479770  ,
481396  ,
483033  ,
484682  ,
486342  ,
488013  ,
489696  ,
491390  ,
493097  ,
494815  ,
496545  ,
498287  ,
500042  ,
501809  ,
503588  ,
505380  ,
507185  ,
509003  ,
510834  ,
512678  ,
514536  ,
516407  ,
518291  ,
520190  ,
522102  ,
524029  ,
525970  ,
527925  ,
529895  ,
531880  ,
533879  ,
535894  ,
537924  ,
539969  ,
542030  ,
544107  ,
546199  ,
548308  ,
550433  ,
552575  ,
554734  ,
556909  ,
559102  ,
561312  ,
563539  ,
565784  ,
568047  ,
570329  ,
572628  ,
574947  ,
577284  ,
579640  ,
582016  ,
584411  ,
586826  ,
589261  ,
591716  ,
594192  ,
596688  ,
599206  ,
601745  ,
604306  ,
606888  ,
609493  ,
612120  ,
614770  ,
617443  ,
620139  ,
622859  ,
625603  ,
628371  ,
631164  ,
633981  ,
636824  ,
639693  ,
642587  ,
645508  ,
648456  ,
651430  ,
654432  ,
657462  ,
660520  ,
663607  ,
666722  ,
669867  ,
673042  ,
676247  ,
679482  ,
682749  ,
686048  ,
689378  ,
692741  ,
696136  ,
699566  ,
703029  ,
706527  ,
710059  ,
713627  ,
717231  ,
720872  ,
724550  ,
728266  ,
732020  ,
735813  ,
739645  ,
743517  ,
747431  ,
751385  ,
755382  ,
759422  ,
763504  ,
767632  ,
771803  ,
776021  ,
780285  ,
784596  ,
788955  ,
793362  ,
797819  ,
802327  ,
806885  ,
811496  ,
816160  ,
820878  ,
825650  ,
830479  ,
835364  ,
840307  ,
845309  ,
850370  ,
855493  ,
860678  ,
865926  ,
871238  ,
876616  ,
882061  ,
887574  ,
893156  ,
898809  ,
904534  ,
910332  ,
916205  ,
922155  ,
928182  ,
934288  ,
940476  ,
946746  ,
953100  ,
959539  ,
966067  ,
972684  ,
979392  ,
986193  ,
993090  ,
1000083 ,
1007176 ,
1014370 ,
1021668 ,
1029071 ,
1036583 ,
1044205 ,
1051940 ,
1059790 ,
1067758 ,
1075847 ,
1084060 ,
1092399 ,
1100867 ,
1109467 ,
1118203 ,
1127078 ,
1136095 ,
1145257 ,
1154568 ,
1164031 ,
1173652 ,
1183432 ,
1193377 ,
1203490 ,
1213776 ,
1224240 ,
1234886 ,
1245718 ,
1256742 ,
1267963 ,
1279386 ,
1291017 ,
1302861 ,
1314924 ,
1327213 ,
1339734 ,
1352494 ,
1365498 ,
1378756 ,
1392273 ,
1406058 ,
1420118 ,
1434463 ,
1449100 ,
1464040 ,
1479290 ,
1494861 ,
1510764 ,
1527009 ,
1543607 ,
1560570 ,
1577909 ,
1595639 ,
1613771 ,
1632320 ,
1651300 ,
1670727 ,
1690617 ,
1710986 ,
1731852 ,
1753233 ,
1775148 ,
1797618 ,
1820665 ,
1844310 ,
1868577 ,
1893491 ,
1919079 ,
1945368 ,
1972387 ,
2000167 ,
2028740 ,
2058143 ,
2088409 ,
2119580 ,
2151694 ,
2184797 ,
2218935 ,
2254156 ,
2290513 ,
2328063 ,
2366864 ,
2406980 ,
2448480 ,
2491436 ,
2535926 ,
2582033 ,
2629849 ,
2679469 ,
2730997 ,
2784546 
//2840237
};

