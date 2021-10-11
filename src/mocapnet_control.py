#! /usr/bin/env python
import rospy, time, math, cv2, sys
import actionlib
import copy
import whole_body_ik_msgs.msg





def getMocapNETJointNames():
 bn=list()
 #---------------------------------------------------
 bn.append("hip_Xposition") # 0
 bn.append("hip_Yposition") # 1
 bn.append("hip_Zposition") # 2
 bn.append("hip_Zrotation") # 3
 bn.append("hip_Yrotation") # 4
 bn.append("hip_Xrotation") # 5
 bn.append("abdomen_Zrotation") # 6
 bn.append("abdomen_Xrotation") # 7
 bn.append("abdomen_Yrotation") # 8
 bn.append("chest_Zrotation") # 9
 bn.append("chest_Xrotation") # 10
 bn.append("chest_Yrotation") # 11
 bn.append("neck_Zrotation") # 12
 bn.append("neck_Xrotation") # 13
 bn.append("neck_Yrotation") # 14
 bn.append("neck1_Zrotation") # 15
 bn.append("neck1_Xrotation") # 16
 bn.append("neck1_Yrotation") # 17
 bn.append("head_Zrotation") # 18
 bn.append("head_Xrotation") # 19
 bn.append("head_Yrotation") # 20
 bn.append("__jaw_Zrotation") # 21
 bn.append("__jaw_Xrotation") # 22
 bn.append("__jaw_Yrotation") # 23
 bn.append("jaw_Zrotation") # 24
 bn.append("jaw_Xrotation") # 25
 bn.append("jaw_Yrotation") # 26
 bn.append("special04_Zrotation") # 27
 bn.append("special04_Xrotation") # 28
 bn.append("special04_Yrotation") # 29
 bn.append("oris02_Zrotation") # 30
 bn.append("oris02_Xrotation") # 31
 bn.append("oris02_Yrotation") # 32
 bn.append("oris01_Zrotation") # 33
 bn.append("oris01_Xrotation") # 34
 bn.append("oris01_Yrotation") # 35
 bn.append("oris06.l_Zrotation") # 36
 bn.append("oris06.l_Xrotation") # 37
 bn.append("oris06.l_Yrotation") # 38
 bn.append("oris07.l_Zrotation") # 39
 bn.append("oris07.l_Xrotation") # 40
 bn.append("oris07.l_Yrotation") # 41
 bn.append("oris06.r_Zrotation") # 42
 bn.append("oris06.r_Xrotation") # 43
 bn.append("oris06.r_Yrotation") # 44
 bn.append("oris07.r_Zrotation") # 45
 bn.append("oris07.r_Xrotation") # 46
 bn.append("oris07.r_Yrotation") # 47
 bn.append("tongue00_Zrotation") # 48
 bn.append("tongue00_Xrotation") # 49
 bn.append("tongue00_Yrotation") # 50
 bn.append("tongue01_Zrotation") # 51
 bn.append("tongue01_Xrotation") # 52
 bn.append("tongue01_Yrotation") # 53
 bn.append("tongue02_Zrotation") # 54
 bn.append("tongue02_Xrotation") # 55
 bn.append("tongue02_Yrotation") # 56
 bn.append("tongue03_Zrotation") # 57
 bn.append("tongue03_Xrotation") # 58
 bn.append("tongue03_Yrotation") # 59
 bn.append("__tongue04_Zrotation") # 60
 bn.append("__tongue04_Xrotation") # 61
 bn.append("__tongue04_Yrotation") # 62
 bn.append("tongue04_Zrotation") # 63
 bn.append("tongue04_Xrotation") # 64
 bn.append("tongue04_Yrotation") # 65
 bn.append("tongue07.l_Zrotation") # 66
 bn.append("tongue07.l_Xrotation") # 67
 bn.append("tongue07.l_Yrotation") # 68
 bn.append("tongue07.r_Zrotation") # 69
 bn.append("tongue07.r_Xrotation") # 70
 bn.append("tongue07.r_Yrotation") # 71
 bn.append("tongue06.l_Zrotation") # 72
 bn.append("tongue06.l_Xrotation") # 73
 bn.append("tongue06.l_Yrotation") # 74
 bn.append("tongue06.r_Zrotation") # 75
 bn.append("tongue06.r_Xrotation") # 76
 bn.append("tongue06.r_Yrotation") # 77
 bn.append("tongue05.l_Zrotation") # 78
 bn.append("tongue05.l_Xrotation") # 79
 bn.append("tongue05.l_Yrotation") # 80
 bn.append("tongue05.r_Zrotation") # 81
 bn.append("tongue05.r_Xrotation") # 82
 bn.append("tongue05.r_Yrotation") # 83
 bn.append("__levator02.l_Zrotation") # 84
 bn.append("__levator02.l_Xrotation") # 85
 bn.append("__levator02.l_Yrotation") # 86
 bn.append("levator02.l_Zrotation") # 87
 bn.append("levator02.l_Xrotation") # 88
 bn.append("levator02.l_Yrotation") # 89
 bn.append("levator03.l_Zrotation") # 90
 bn.append("levator03.l_Xrotation") # 91
 bn.append("levator03.l_Yrotation") # 92
 bn.append("levator04.l_Zrotation") # 93
 bn.append("levator04.l_Xrotation") # 94
 bn.append("levator04.l_Yrotation") # 95
 bn.append("levator05.l_Zrotation") # 96
 bn.append("levator05.l_Xrotation") # 97
 bn.append("levator05.l_Yrotation") # 98
 bn.append("__levator02.r_Zrotation") # 99
 bn.append("__levator02.r_Xrotation") # 100
 bn.append("__levator02.r_Yrotation") # 101
 bn.append("levator02.r_Zrotation") # 102
 bn.append("levator02.r_Xrotation") # 103
 bn.append("levator02.r_Yrotation") # 104
 bn.append("levator03.r_Zrotation") # 105
 bn.append("levator03.r_Xrotation") # 106
 bn.append("levator03.r_Yrotation") # 107
 bn.append("levator04.r_Zrotation") # 108
 bn.append("levator04.r_Xrotation") # 109
 bn.append("levator04.r_Yrotation") # 110
 bn.append("levator05.r_Zrotation") # 111
 bn.append("levator05.r_Xrotation") # 112
 bn.append("levator05.r_Yrotation") # 113
 bn.append("__special01_Zrotation") # 114
 bn.append("__special01_Xrotation") # 115
 bn.append("__special01_Yrotation") # 116
 bn.append("special01_Zrotation") # 117
 bn.append("special01_Xrotation") # 118
 bn.append("special01_Yrotation") # 119
 bn.append("oris04.l_Zrotation") # 120
 bn.append("oris04.l_Xrotation") # 121
 bn.append("oris04.l_Yrotation") # 122
 bn.append("oris03.l_Zrotation") # 123
 bn.append("oris03.l_Xrotation") # 124
 bn.append("oris03.l_Yrotation") # 125
 bn.append("oris04.r_Zrotation") # 126
 bn.append("oris04.r_Xrotation") # 127
 bn.append("oris04.r_Yrotation") # 128
 bn.append("oris03.r_Zrotation") # 129
 bn.append("oris03.r_Xrotation") # 130
 bn.append("oris03.r_Yrotation") # 131
 bn.append("oris06_Zrotation") # 132
 bn.append("oris06_Xrotation") # 133
 bn.append("oris06_Yrotation") # 134
 bn.append("oris05_Zrotation") # 135
 bn.append("oris05_Xrotation") # 136
 bn.append("oris05_Yrotation") # 137
 bn.append("__special03_Zrotation") # 138
 bn.append("__special03_Xrotation") # 139
 bn.append("__special03_Yrotation") # 140
 bn.append("special03_Zrotation") # 141
 bn.append("special03_Xrotation") # 142
 bn.append("special03_Yrotation") # 143
 bn.append("__levator06.l_Zrotation") # 144
 bn.append("__levator06.l_Xrotation") # 145
 bn.append("__levator06.l_Yrotation") # 146
 bn.append("levator06.l_Zrotation") # 147
 bn.append("levator06.l_Xrotation") # 148
 bn.append("levator06.l_Yrotation") # 149
 bn.append("__levator06.r_Zrotation") # 150
 bn.append("__levator06.r_Xrotation") # 151
 bn.append("__levator06.r_Yrotation") # 152
 bn.append("levator06.r_Zrotation") # 153
 bn.append("levator06.r_Xrotation") # 154
 bn.append("levator06.r_Yrotation") # 155
 bn.append("special06.l_Zrotation") # 156
 bn.append("special06.l_Xrotation") # 157
 bn.append("special06.l_Yrotation") # 158
 bn.append("special05.l_Zrotation") # 159
 bn.append("special05.l_Xrotation") # 160
 bn.append("special05.l_Yrotation") # 161
 bn.append("eye.l_Zrotation") # 162
 bn.append("eye.l_Xrotation") # 163
 bn.append("eye.l_Yrotation") # 164
 bn.append("orbicularis03.l_Zrotation") # 165
 bn.append("orbicularis03.l_Xrotation") # 166
 bn.append("orbicularis03.l_Yrotation") # 167
 bn.append("orbicularis04.l_Zrotation") # 168
 bn.append("orbicularis04.l_Xrotation") # 169
 bn.append("orbicularis04.l_Yrotation") # 170
 bn.append("special06.r_Zrotation") # 171
 bn.append("special06.r_Xrotation") # 172
 bn.append("special06.r_Yrotation") # 173
 bn.append("special05.r_Zrotation") # 174
 bn.append("special05.r_Xrotation") # 175
 bn.append("special05.r_Yrotation") # 176
 bn.append("eye.r_Zrotation") # 177
 bn.append("eye.r_Xrotation") # 178
 bn.append("eye.r_Yrotation") # 179
 bn.append("orbicularis03.r_Zrotation") # 180
 bn.append("orbicularis03.r_Xrotation") # 181
 bn.append("orbicularis03.r_Yrotation") # 182
 bn.append("orbicularis04.r_Zrotation") # 183
 bn.append("orbicularis04.r_Xrotation") # 184
 bn.append("orbicularis04.r_Yrotation") # 185
 bn.append("__temporalis01.l_Zrotation") # 186
 bn.append("__temporalis01.l_Xrotation") # 187
 bn.append("__temporalis01.l_Yrotation") # 188
 bn.append("temporalis01.l_Zrotation") # 189
 bn.append("temporalis01.l_Xrotation") # 190
 bn.append("temporalis01.l_Yrotation") # 191
 bn.append("oculi02.l_Zrotation") # 192
 bn.append("oculi02.l_Xrotation") # 193
 bn.append("oculi02.l_Yrotation") # 194
 bn.append("oculi01.l_Zrotation") # 195
 bn.append("oculi01.l_Xrotation") # 196
 bn.append("oculi01.l_Yrotation") # 197
 bn.append("__temporalis01.r_Zrotation") # 198
 bn.append("__temporalis01.r_Xrotation") # 199
 bn.append("__temporalis01.r_Yrotation") # 200
 bn.append("temporalis01.r_Zrotation") # 201
 bn.append("temporalis01.r_Xrotation") # 202
 bn.append("temporalis01.r_Yrotation") # 203
 bn.append("oculi02.r_Zrotation") # 204
 bn.append("oculi02.r_Xrotation") # 205
 bn.append("oculi02.r_Yrotation") # 206
 bn.append("oculi01.r_Zrotation") # 207
 bn.append("oculi01.r_Xrotation") # 208
 bn.append("oculi01.r_Yrotation") # 209
 bn.append("__temporalis02.l_Zrotation") # 210
 bn.append("__temporalis02.l_Xrotation") # 211
 bn.append("__temporalis02.l_Yrotation") # 212
 bn.append("temporalis02.l_Zrotation") # 213
 bn.append("temporalis02.l_Xrotation") # 214
 bn.append("temporalis02.l_Yrotation") # 215
 bn.append("risorius02.l_Zrotation") # 216
 bn.append("risorius02.l_Xrotation") # 217
 bn.append("risorius02.l_Yrotation") # 218
 bn.append("risorius03.l_Zrotation") # 219
 bn.append("risorius03.l_Xrotation") # 220
 bn.append("risorius03.l_Yrotation") # 221
 bn.append("__temporalis02.r_Zrotation") # 222
 bn.append("__temporalis02.r_Xrotation") # 223
 bn.append("__temporalis02.r_Yrotation") # 224
 bn.append("temporalis02.r_Zrotation") # 225
 bn.append("temporalis02.r_Xrotation") # 226
 bn.append("temporalis02.r_Yrotation") # 227
 bn.append("risorius02.r_Zrotation") # 228
 bn.append("risorius02.r_Xrotation") # 229
 bn.append("risorius02.r_Yrotation") # 230
 bn.append("risorius03.r_Zrotation") # 231
 bn.append("risorius03.r_Xrotation") # 232
 bn.append("risorius03.r_Yrotation") # 233
 bn.append("rcollar_Zrotation") # 234
 bn.append("rcollar_Xrotation") # 235
 bn.append("rcollar_Yrotation") # 236
 bn.append("rshoulder_Zrotation") # 237
 bn.append("rshoulder_Xrotation") # 238
 bn.append("rshoulder_Yrotation") # 239
 bn.append("relbow_Zrotation") # 240
 bn.append("relbow_Xrotation") # 241
 bn.append("relbow_Yrotation") # 242
 bn.append("rhand_Zrotation") # 243
 bn.append("rhand_Xrotation") # 244
 bn.append("rhand_Yrotation") # 245
 bn.append("metacarpal1.r_Zrotation") # 246
 bn.append("metacarpal1.r_Xrotation") # 247
 bn.append("metacarpal1.r_Yrotation") # 248
 bn.append("finger2-1.r_Zrotation") # 249
 bn.append("finger2-1.r_Xrotation") # 250
 bn.append("finger2-1.r_Yrotation") # 251
 bn.append("finger2-2.r_Zrotation") # 252
 bn.append("finger2-2.r_Xrotation") # 253
 bn.append("finger2-2.r_Yrotation") # 254
 bn.append("finger2-3.r_Zrotation") # 255
 bn.append("finger2-3.r_Xrotation") # 256
 bn.append("finger2-3.r_Yrotation") # 257
 bn.append("metacarpal2.r_Zrotation") # 258
 bn.append("metacarpal2.r_Xrotation") # 259
 bn.append("metacarpal2.r_Yrotation") # 260
 bn.append("finger3-1.r_Zrotation") # 261
 bn.append("finger3-1.r_Xrotation") # 262
 bn.append("finger3-1.r_Yrotation") # 263
 bn.append("finger3-2.r_Zrotation") # 264
 bn.append("finger3-2.r_Xrotation") # 265
 bn.append("finger3-2.r_Yrotation") # 266
 bn.append("finger3-3.r_Zrotation") # 267
 bn.append("finger3-3.r_Xrotation") # 268
 bn.append("finger3-3.r_Yrotation") # 269
 bn.append("__metacarpal3.r_Zrotation") # 270
 bn.append("__metacarpal3.r_Xrotation") # 271
 bn.append("__metacarpal3.r_Yrotation") # 272
 bn.append("metacarpal3.r_Zrotation") # 273
 bn.append("metacarpal3.r_Xrotation") # 274
 bn.append("metacarpal3.r_Yrotation") # 275
 bn.append("finger4-1.r_Zrotation") # 276
 bn.append("finger4-1.r_Xrotation") # 277
 bn.append("finger4-1.r_Yrotation") # 278
 bn.append("finger4-2.r_Zrotation") # 279
 bn.append("finger4-2.r_Xrotation") # 280
 bn.append("finger4-2.r_Yrotation") # 281
 bn.append("finger4-3.r_Zrotation") # 282
 bn.append("finger4-3.r_Xrotation") # 283
 bn.append("finger4-3.r_Yrotation") # 284
 bn.append("__metacarpal4.r_Zrotation") # 285
 bn.append("__metacarpal4.r_Xrotation") # 286
 bn.append("__metacarpal4.r_Yrotation") # 287
 bn.append("metacarpal4.r_Zrotation") # 288
 bn.append("metacarpal4.r_Xrotation") # 289
 bn.append("metacarpal4.r_Yrotation") # 290
 bn.append("finger5-1.r_Zrotation") # 291
 bn.append("finger5-1.r_Xrotation") # 292
 bn.append("finger5-1.r_Yrotation") # 293
 bn.append("finger5-2.r_Zrotation") # 294
 bn.append("finger5-2.r_Xrotation") # 295
 bn.append("finger5-2.r_Yrotation") # 296
 bn.append("finger5-3.r_Zrotation") # 297
 bn.append("finger5-3.r_Xrotation") # 298
 bn.append("finger5-3.r_Yrotation") # 299
 bn.append("rthumbBase_Zrotation") # 300
 bn.append("rthumbBase_Xrotation") # 301
 bn.append("rthumbBase_Yrotation") # 302
 bn.append("rthumb_Zrotation") # 303
 bn.append("rthumb_Xrotation") # 304
 bn.append("rthumb_Yrotation") # 305
 bn.append("finger1-2.r_Zrotation") # 306
 bn.append("finger1-2.r_Xrotation") # 307
 bn.append("finger1-2.r_Yrotation") # 308
 bn.append("finger1-3.r_Zrotation") # 309
 bn.append("finger1-3.r_Xrotation") # 310
 bn.append("finger1-3.r_Yrotation") # 311
 bn.append("lcollar_Zrotation") # 312
 bn.append("lcollar_Xrotation") # 313
 bn.append("lcollar_Yrotation") # 314
 bn.append("lshoulder_Zrotation") # 315
 bn.append("lshoulder_Xrotation") # 316
 bn.append("lshoulder_Yrotation") # 317
 bn.append("lelbow_Zrotation") # 318
 bn.append("lelbow_Xrotation") # 319
 bn.append("lelbow_Yrotation") # 320
 bn.append("lhand_Zrotation") # 321
 bn.append("lhand_Xrotation") # 322
 bn.append("lhand_Yrotation") # 323
 bn.append("metacarpal1.l_Zrotation") # 324
 bn.append("metacarpal1.l_Xrotation") # 325
 bn.append("metacarpal1.l_Yrotation") # 326
 bn.append("finger2-1.l_Zrotation") # 327
 bn.append("finger2-1.l_Xrotation") # 328
 bn.append("finger2-1.l_Yrotation") # 329
 bn.append("finger2-2.l_Zrotation") # 330
 bn.append("finger2-2.l_Xrotation") # 331
 bn.append("finger2-2.l_Yrotation") # 332
 bn.append("finger2-3.l_Zrotation") # 333
 bn.append("finger2-3.l_Xrotation") # 334
 bn.append("finger2-3.l_Yrotation") # 335
 bn.append("metacarpal2.l_Zrotation") # 336
 bn.append("metacarpal2.l_Xrotation") # 337
 bn.append("metacarpal2.l_Yrotation") # 338
 bn.append("finger3-1.l_Zrotation") # 339
 bn.append("finger3-1.l_Xrotation") # 340
 bn.append("finger3-1.l_Yrotation") # 341
 bn.append("finger3-2.l_Zrotation") # 342
 bn.append("finger3-2.l_Xrotation") # 343
 bn.append("finger3-2.l_Yrotation") # 344
 bn.append("finger3-3.l_Zrotation") # 345
 bn.append("finger3-3.l_Xrotation") # 346
 bn.append("finger3-3.l_Yrotation") # 347
 bn.append("__metacarpal3.l_Zrotation") # 348
 bn.append("__metacarpal3.l_Xrotation") # 349
 bn.append("__metacarpal3.l_Yrotation") # 350
 bn.append("metacarpal3.l_Zrotation") # 351
 bn.append("metacarpal3.l_Xrotation") # 352
 bn.append("metacarpal3.l_Yrotation") # 353
 bn.append("finger4-1.l_Zrotation") # 354
 bn.append("finger4-1.l_Xrotation") # 355
 bn.append("finger4-1.l_Yrotation") # 356
 bn.append("finger4-2.l_Zrotation") # 357
 bn.append("finger4-2.l_Xrotation") # 358
 bn.append("finger4-2.l_Yrotation") # 359
 bn.append("finger4-3.l_Zrotation") # 360
 bn.append("finger4-3.l_Xrotation") # 361
 bn.append("finger4-3.l_Yrotation") # 362
 bn.append("__metacarpal4.l_Zrotation") # 363
 bn.append("__metacarpal4.l_Xrotation") # 364
 bn.append("__metacarpal4.l_Yrotation") # 365
 bn.append("metacarpal4.l_Zrotation") # 366
 bn.append("metacarpal4.l_Xrotation") # 367
 bn.append("metacarpal4.l_Yrotation") # 368
 bn.append("finger5-1.l_Zrotation") # 369
 bn.append("finger5-1.l_Xrotation") # 370
 bn.append("finger5-1.l_Yrotation") # 371
 bn.append("finger5-2.l_Zrotation") # 372
 bn.append("finger5-2.l_Xrotation") # 373
 bn.append("finger5-2.l_Yrotation") # 374
 bn.append("finger5-3.l_Zrotation") # 375
 bn.append("finger5-3.l_Xrotation") # 376
 bn.append("finger5-3.l_Yrotation") # 377
 bn.append("lthumbBase_Zrotation") # 378
 bn.append("lthumbBase_Xrotation") # 379
 bn.append("lthumbBase_Yrotation") # 380
 bn.append("lthumb_Zrotation") # 381
 bn.append("lthumb_Xrotation") # 382
 bn.append("lthumb_Yrotation") # 383
 bn.append("finger1-2.l_Zrotation") # 384
 bn.append("finger1-2.l_Xrotation") # 385
 bn.append("finger1-2.l_Yrotation") # 386
 bn.append("finger1-3.l_Zrotation") # 387
 bn.append("finger1-3.l_Xrotation") # 388
 bn.append("finger1-3.l_Yrotation") # 389
 bn.append("rbuttock_Zrotation") # 390
 bn.append("rbuttock_Xrotation") # 391
 bn.append("rbuttock_Yrotation") # 392
 bn.append("rhip_Zrotation") # 393
 bn.append("rhip_Xrotation") # 394
 bn.append("rhip_Yrotation") # 395
 bn.append("rknee_Zrotation") # 396
 bn.append("rknee_Xrotation") # 397
 bn.append("rknee_Yrotation") # 398
 bn.append("rfoot_Zrotation") # 399
 bn.append("rfoot_Xrotation") # 400
 bn.append("rfoot_Yrotation") # 401
 bn.append("toe1-1.r_Zrotation") # 402
 bn.append("toe1-1.r_Xrotation") # 403
 bn.append("toe1-1.r_Yrotation") # 404
 bn.append("toe1-2.r_Zrotation") # 405
 bn.append("toe1-2.r_Xrotation") # 406
 bn.append("toe1-2.r_Yrotation") # 407
 bn.append("toe2-1.r_Zrotation") # 408
 bn.append("toe2-1.r_Xrotation") # 409
 bn.append("toe2-1.r_Yrotation") # 410
 bn.append("toe2-2.r_Zrotation") # 411
 bn.append("toe2-2.r_Xrotation") # 412
 bn.append("toe2-2.r_Yrotation") # 413
 bn.append("toe2-3.r_Zrotation") # 414
 bn.append("toe2-3.r_Xrotation") # 415
 bn.append("toe2-3.r_Yrotation") # 416
 bn.append("toe3-1.r_Zrotation") # 417
 bn.append("toe3-1.r_Xrotation") # 418
 bn.append("toe3-1.r_Yrotation") # 419
 bn.append("toe3-2.r_Zrotation") # 420
 bn.append("toe3-2.r_Xrotation") # 421
 bn.append("toe3-2.r_Yrotation") # 422
 bn.append("toe3-3.r_Zrotation") # 423
 bn.append("toe3-3.r_Xrotation") # 424
 bn.append("toe3-3.r_Yrotation") # 425
 bn.append("toe4-1.r_Zrotation") # 426
 bn.append("toe4-1.r_Xrotation") # 427
 bn.append("toe4-1.r_Yrotation") # 428
 bn.append("toe4-2.r_Zrotation") # 429
 bn.append("toe4-2.r_Xrotation") # 430
 bn.append("toe4-2.r_Yrotation") # 431
 bn.append("toe4-3.r_Zrotation") # 432
 bn.append("toe4-3.r_Xrotation") # 433
 bn.append("toe4-3.r_Yrotation") # 434
 bn.append("toe5-1.r_Zrotation") # 435
 bn.append("toe5-1.r_Xrotation") # 436
 bn.append("toe5-1.r_Yrotation") # 437
 bn.append("toe5-2.r_Zrotation") # 438
 bn.append("toe5-2.r_Xrotation") # 439
 bn.append("toe5-2.r_Yrotation") # 440
 bn.append("toe5-3.r_Zrotation") # 441
 bn.append("toe5-3.r_Xrotation") # 442
 bn.append("toe5-3.r_Yrotation") # 443
 bn.append("lbuttock_Zrotation") # 444
 bn.append("lbuttock_Xrotation") # 445
 bn.append("lbuttock_Yrotation") # 446
 bn.append("lhip_Zrotation") # 447
 bn.append("lhip_Xrotation") # 448
 bn.append("lhip_Yrotation") # 449
 bn.append("lknee_Zrotation") # 450
 bn.append("lknee_Xrotation") # 451
 bn.append("lknee_Yrotation") # 452
 bn.append("lfoot_Zrotation") # 453
 bn.append("lfoot_Xrotation") # 454
 bn.append("lfoot_Yrotation") # 455
 bn.append("toe1-1.l_Zrotation") # 456
 bn.append("toe1-1.l_Xrotation") # 457
 bn.append("toe1-1.l_Yrotation") # 458
 bn.append("toe1-2.l_Zrotation") # 459
 bn.append("toe1-2.l_Xrotation") # 460
 bn.append("toe1-2.l_Yrotation") # 461
 bn.append("toe2-1.l_Zrotation") # 462
 bn.append("toe2-1.l_Xrotation") # 463
 bn.append("toe2-1.l_Yrotation") # 464
 bn.append("toe2-2.l_Zrotation") # 465
 bn.append("toe2-2.l_Xrotation") # 466
 bn.append("toe2-2.l_Yrotation") # 467
 bn.append("toe2-3.l_Zrotation") # 468
 bn.append("toe2-3.l_Xrotation") # 469
 bn.append("toe2-3.l_Yrotation") # 470
 bn.append("toe3-1.l_Zrotation") # 471
 bn.append("toe3-1.l_Xrotation") # 472
 bn.append("toe3-1.l_Yrotation") # 473
 bn.append("toe3-2.l_Zrotation") # 474
 bn.append("toe3-2.l_Xrotation") # 475
 bn.append("toe3-2.l_Yrotation") # 476
 bn.append("toe3-3.l_Zrotation") # 477
 bn.append("toe3-3.l_Xrotation") # 478
 bn.append("toe3-3.l_Yrotation") # 479
 bn.append("toe4-1.l_Zrotation") # 480
 bn.append("toe4-1.l_Xrotation") # 481
 bn.append("toe4-1.l_Yrotation") # 482
 bn.append("toe4-2.l_Zrotation") # 483
 bn.append("toe4-2.l_Xrotation") # 484
 bn.append("toe4-2.l_Yrotation") # 485
 bn.append("toe4-3.l_Zrotation") # 486
 bn.append("toe4-3.l_Xrotation") # 487
 bn.append("toe4-3.l_Yrotation") # 488
 bn.append("toe5-1.l_Zrotation") # 489
 bn.append("toe5-1.l_Xrotation") # 490
 bn.append("toe5-1.l_Yrotation") # 491
 bn.append("toe5-2.l_Zrotation") # 492
 bn.append("toe5-2.l_Xrotation") # 493
 bn.append("toe5-2.l_Yrotation") # 494
 bn.append("toe5-3.l_Zrotation") # 495
 bn.append("toe5-3.l_Xrotation") # 496
 bn.append("toe5-3.l_Yrotation") # 497
 return bn
#---------------------------------------------------
 

def degreesToRadians(degrees):
  return degrees * math.pi / 180.0



from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
CoM = False
LLeg = False
RLeg = False
odom = False
joints = False
LLeg_odom = Odometry()
RLeg_odom = Odometry()
CoM_odom = Odometry()
base_odom = Odometry()
joint_state = JointState()



def JointStatecallback(data):
    global joint_state
    joint_state = data
    global joints 
    joints = True

def LLegcallback(data):
    global LLeg_odom
    LLeg_odom = data
    global LLeg 
    LLeg = True

def RLegcallback(data):
    global RLeg_odom
    RLeg_odom = data
    global RLeg
    RLeg = True

def CoMcallback(data):
    global CoM_odom
    CoM_odom = data
    global CoM 
    CoM = True

def OdomCallback(data):
    global base_odom
    base_odom = data
    global odom
    odom = True 


from std_msgs.msg import Float32MultiArray
mocapNETLabels = getMocapNETJointNames()

mocapNETPose = list()
mocapNETPoseExists = 0

#define function/functions to provide the required functionality
def mnet_new_pose_callback(msg):
    #make_something_here_with msg.data
    #rospy.loginfo("I heard %s", msg.data)
    mocapNETPose=msg.data[i]
    mocapNETPoseExists=1
    for i in range(0,len(msg.data)):
        print (i,"[",mocapNETLabels[i],"] = ",msg.data[i]," ")


def action_client():
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('/nao_raisim_ros/whole_body_control', whole_body_ik_msgs.msg.HumanoidAction)



    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    print("WBC Server is UP")
    # Creates a goal to send to the action server.
    goal = whole_body_ik_msgs.msg.HumanoidGoal()

    goal.odom = base_odom
    goal.joint_state = joint_state

    goal.dt = 0.01 #Hardware Control Cycle of NAO

    #Do Not Change
    dof_gain = 0.6
    dof_weight = 5.0e-05
    goal.Joints = []
    tmp_dof_task = whole_body_ik_msgs.msg.DOFTask()
    tmp_dof_task.weight = dof_weight
    tmp_dof_task.gain = dof_gain
    #HEAD
    tmp_dof_task.desired_angle = 0.0
    tmp_dof_task.name = "HeadYaw"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = 0.0
    tmp_dof_task.name = "HeadPitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    

    
    #LEFT LEG
    tmp_dof_task.desired_angle = 0.0
    tmp_dof_task.name = "LHipYawPitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = 0.0
    tmp_dof_task.name = "LHipRoll"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = -0.3976
    tmp_dof_task.name = "LHipPitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle =  0.85
    tmp_dof_task.name = "LKneePitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = -0.4427
    tmp_dof_task.name = "LAnklePitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = -0.009
    tmp_dof_task.name = "LAnkleRoll"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))


    #RIGHT LEG
    tmp_dof_task.desired_angle = 0.0
    tmp_dof_task.name = "RHipYawPitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = 0.0
    tmp_dof_task.name = "RHipRoll"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = -0.3976
    tmp_dof_task.name = "RHipPitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle =  0.85
    tmp_dof_task.name = "RKneePitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = -0.4427
    tmp_dof_task.name = "RAnklePitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = -0.009
    tmp_dof_task.name = "RAnkleRoll"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
  
    #LEFT ARM
    tmp_dof_task.desired_angle =  1.5
    tmp_dof_task.name = "LShoulderPitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    #------------------------------------------------- 
    tmp_dof_task.desired_angle = 0.15
    tmp_dof_task.name = "LShoulderRoll"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    #------------------------------------------------- 
    tmp_dof_task.desired_angle = 0
    tmp_dof_task.name = "LElbowYaw"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    #------------------------------------------------- 
    tmp_dof_task.desired_angle =  -0.0349066
    tmp_dof_task.name = "LElbowRoll"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    #------------------------------------------------- 
    tmp_dof_task.desired_angle = -1.5
    tmp_dof_task.name = "LWristYaw"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    #------------------------------------------------- 
    tmp_dof_task.desired_angle = 0
    tmp_dof_task.name = "LHand"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    #------------------------------------------------- 

    #RIGHT ARM
    tmp_dof_task.desired_angle =  1.5
    if (mocapNETPoseExists):
      tmp_dof_task.desired_angle =  degreesToRadians(mocapNETPose[237]) #rshoulder_Zrotation
    tmp_dof_task.name = "RShoulderPitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    #------------------------------------------------- 
    tmp_dof_task.desired_angle = -0.15
    if (mocapNETPoseExists):
      tmp_dof_task.desired_angle =  degreesToRadians(mocapNETPose[238]) #rshoulder_Xrotation
    tmp_dof_task.name = "RShoulderRoll"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    #------------------------------------------------- 
    tmp_dof_task.desired_angle =  0
    tmp_dof_task.name = "RElbowYaw"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    #------------------------------------------------- 
    tmp_dof_task.desired_angle = 0.0349066
    tmp_dof_task.name = "RElbowRoll"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    #------------------------------------------------- 
    tmp_dof_task.desired_angle = 1.5
    tmp_dof_task.name = "RWristYaw"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    #------------------------------------------------- 
    tmp_dof_task.desired_angle = 0
    tmp_dof_task.name = "RHand"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    #------------------------------------------------- 



    #Set Linear Task for CoM
    goal.CoM.linear_task.desired_position.x = CoM_odom.pose.pose.position.x
    goal.CoM.linear_task.desired_position.y = CoM_odom.pose.pose.position.y
    goal.CoM.linear_task.desired_position.z = CoM_odom.pose.pose.position.z
    goal.CoM.linear_task.desired_linear_velocity.x = CoM_odom.twist.twist.linear.x
    goal.CoM.linear_task.desired_linear_velocity.y = CoM_odom.twist.twist.linear.y
    goal.CoM.linear_task.desired_linear_velocity.z = CoM_odom.twist.twist.linear.z
    goal.CoM.linear_task.weight = 5.0e-1
    goal.CoM.linear_task.gain  = 0.35


    #Set Angular Task for Torso
    goal.Torso.angular_task.desired_angular_velocity.x = 0.0
    goal.Torso.angular_task.desired_angular_velocity.y = 0.0
    goal.Torso.angular_task.desired_angular_velocity.z = 0.0
    goal.Torso.angular_task.desired_orientation.x = 0.0
    goal.Torso.angular_task.desired_orientation.y = 0.0
    goal.Torso.angular_task.desired_orientation.z = 0.0
    goal.Torso.angular_task.desired_orientation.w = 1.0
    goal.Torso.angular_task.weight = 1.0e-3
    goal.Torso.angular_task.gain  = 0.2


    #Set Leg Task to keep their pose w.r.t ground
    goal.LLeg.linear_task.desired_position.x = LLeg_odom.pose.pose.position.x
    goal.LLeg.linear_task.desired_position.y = LLeg_odom.pose.pose.position.y
    goal.LLeg.linear_task.desired_position.z = LLeg_odom.pose.pose.position.z
    goal.LLeg.linear_task.desired_linear_velocity.x = 0.000
    goal.LLeg.linear_task.desired_linear_velocity.y = 0.000
    goal.LLeg.linear_task.desired_linear_velocity.z = 0.000
    goal.LLeg.linear_task.weight = 2.0
    goal.LLeg.linear_task.gain  = 0.35

    goal.LLeg.angular_task.desired_angular_velocity.x = 0.0
    goal.LLeg.angular_task.desired_angular_velocity.y = 0.0
    goal.LLeg.angular_task.desired_angular_velocity.z = 0.0
    goal.LLeg.angular_task.desired_orientation.x = LLeg_odom.pose.pose.orientation.x
    goal.LLeg.angular_task.desired_orientation.y = LLeg_odom.pose.pose.orientation.y
    goal.LLeg.angular_task.desired_orientation.z = LLeg_odom.pose.pose.orientation.z
    goal.LLeg.angular_task.desired_orientation.w = LLeg_odom.pose.pose.orientation.w
    goal.LLeg.angular_task.weight = 2.0
    goal.LLeg.angular_task.gain  = 0.2


    goal.RLeg.linear_task.desired_position.x = RLeg_odom.pose.pose.position.x
    goal.RLeg.linear_task.desired_position.y = RLeg_odom.pose.pose.position.y
    goal.RLeg.linear_task.desired_position.z = RLeg_odom.pose.pose.position.z
    goal.RLeg.linear_task.desired_linear_velocity.x = 0.000
    goal.RLeg.linear_task.desired_linear_velocity.y = 0.000
    goal.RLeg.linear_task.desired_linear_velocity.z = 0.000
    goal.RLeg.linear_task.weight = 2.0
    goal.RLeg.linear_task.gain  = 0.35

    goal.RLeg.angular_task.desired_orientation.x = RLeg_odom.pose.pose.orientation.x
    goal.RLeg.angular_task.desired_orientation.y = RLeg_odom.pose.pose.orientation.y
    goal.RLeg.angular_task.desired_orientation.z = RLeg_odom.pose.pose.orientation.z
    goal.RLeg.angular_task.desired_orientation.w = RLeg_odom.pose.pose.orientation.w
    goal.RLeg.angular_task.desired_angular_velocity.x = 0.0
    goal.RLeg.angular_task.desired_angular_velocity.y = 0.0
    goal.RLeg.angular_task.desired_angular_velocity.z = 0.0
    goal.RLeg.angular_task.weight = 2.0
    goal.RLeg.angular_task.gain  = 0.2




    ####################################################################
    ####################################################################
    #ARM  + Head CONTROL IS HERE
    goal.RHand.linear_task.desired_position.x = 0 #Fix this 
    goal.RHand.linear_task.desired_position.y = 0 #Fix this 
    goal.RHand.linear_task.desired_position.z = 0 #Fix this 
    goal.RHand.linear_task.desired_linear_velocity.x = 0.000 #Leave 0 here
    goal.RHand.linear_task.desired_linear_velocity.y = 0.000 #Leave 0 here
    goal.RHand.linear_task.desired_linear_velocity.z = 0.000 #Leave 0 here
    goal.RHand.linear_task.weight = -1 #Fix this 
    goal.RHand.linear_task.gain  = 0.35 #Fix this 


    goal.RHand.angular_task.desired_orientation.x = 0 #Fix this 
    goal.RHand.angular_task.desired_orientation.y = 0 #Fix this 
    goal.RHand.angular_task.desired_orientation.z = 0 #Fix this 
    goal.RHand.angular_task.desired_orientation.w = 0 #Fix this 
    goal.RHand.angular_task.desired_angular_velocity.x = 0.0 #Leave 0 here
    goal.RHand.angular_task.desired_angular_velocity.y = 0.0 #Leave 0 here
    goal.RHand.angular_task.desired_angular_velocity.z = 0.0 #Leave 0 here
    goal.RHand.angular_task.weight = -1 #Fix this 
    goal.RHand.angular_task.gain  = 0.2 #Fix this 

    goal.LHand.linear_task.desired_position.x = 0 #Fix this 
    goal.LHand.linear_task.desired_position.y = 0 #Fix this 
    goal.LHand.linear_task.desired_position.z = 0 #Fix this 
    goal.LHand.linear_task.desired_linear_velocity.x = 0.000 #Leave 0 here
    goal.LHand.linear_task.desired_linear_velocity.y = 0.000 #Leave 0 here
    goal.LHand.linear_task.desired_linear_velocity.z = 0.000 #Leave 0 here
    goal.LHand.linear_task.weight = -1 #Fix this 
    goal.LHand.linear_task.gain  = 0.35 #Fix this 


    goal.LHand.angular_task.desired_orientation.x = 0 #Fix this 
    goal.LHand.angular_task.desired_orientation.y = 0 #Fix this 
    goal.LHand.angular_task.desired_orientation.z = 0 #Fix this 
    goal.LHand.angular_task.desired_orientation.w = 0 #Fix this 
    goal.LHand.angular_task.desired_angular_velocity.x = 0.0 #Leave 0 here
    goal.LHand.angular_task.desired_angular_velocity.y = 0.0 #Leave 0 here
    goal.LHand.angular_task.desired_angular_velocity.z = 0.0 #Leave 0 here
    goal.LHand.angular_task.weight = -1 #Fix this 
    goal.LHand.angular_task.gain  = 0.2 #Fix this 


    #Translating the Head is not adviced
    # goal.Head.linear_task.desired_position.x = 0 #Fix this 
    # goal.Head.linear_task.desired_position.y = 0 #Fix this 
    # goal.Head.linear_task.desired_position.z = 0 #Fix this 
    # goal.Head.linear_task.desired_linear_velocity.x = 0.000 #Leave 0 here
    # goal.Head.linear_task.desired_linear_velocity.y = 0.000 #Leave 0 here
    # goal.Head.linear_task.desired_linear_velocity.z = 0.000 #Leave 0 here
    # goal.Head.linear_task.weight = 0 #Fix this 
    # goal.Head.linear_task.gain  = 0 #Fix this 


    goal.Head.angular_task.desired_orientation.x = 0 #Fix this 
    goal.Head.angular_task.desired_orientation.y = 0 #Fix this 
    goal.Head.angular_task.desired_orientation.z = 1 #Fix this 
    goal.Head.angular_task.desired_orientation.w = 0 #Fix this 
    goal.Head.angular_task.desired_angular_velocity.x = 0.0 #Leave 0 here
    goal.Head.angular_task.desired_angular_velocity.y = 0.0 #Leave 0 here
    goal.Head.angular_task.desired_angular_velocity.z = 0.0 #Leave 0 here
    goal.Head.angular_task.weight = 5.0e-4 #Fix this 
    goal.Head.angular_task.gain  = 0.2 #Fix this 
    ####################################################################
    ####################################################################

    # Sends the goal to the action server.
    client.send_goal(goal)
    print("Goal Sent")
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    print("Result received")
    # Prints out the result of executing the action
    return client.get_result()  

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('mocapnet_robot_control')
        rospy.Subscriber("/nao_raisim_ros/LLeg/odom", Odometry, LLegcallback)
        rospy.Subscriber("/nao_raisim_ros/RLeg/odom", Odometry, RLegcallback)
        rospy.Subscriber("/nao_raisim_ros/CoM", Odometry, CoMcallback)
        rospy.Subscriber("/nao_raisim_ros/odom", Odometry, OdomCallback)
        rospy.Subscriber("/nao_raisim_ros/joint_states", JointState, JointStatecallback)

        #subscribe to a topic using rospy.Subscriber class "std_msgs/Float32MultiArray"
        sub=rospy.Subscriber('/mocapnet_rosnode/bvhFrame', Float32MultiArray , mnet_new_pose_callback)

        rate = rospy.Rate(100) # 10hz
        while not rospy.is_shutdown():
            print("Waiting for Data")
            rate.sleep()
            if(LLeg and RLeg and CoM and odom and joints):
                break

        print("Data Received")
        result = action_client()
        print("Result:", action_client)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
