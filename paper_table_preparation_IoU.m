top = [
2.3178	0.6207
2.2559	0.5926
];

non_top = [
2.0256	0.3907
2.3247	0.3946
];

% using refinement outperforms xx%
refinement_consistency = (top(1,:) - top(2,:)) ./ top(1,:) * 100;
refinement_none_consistency = (non_top(1, :) - non_top(2,:)) ./ non_top(1,:) * 100;
refinement_consistency'
refinement_none_consistency'


% using top-consistency outperforms xx%
SNR_consistency = (top(1,:) - non_top(1,:)) ./ non_top(1,:) * 100;
SR_consistency = (top(2,:) - non_top(2,:)) ./ non_top(2,:) * 100;
SNR_consistency'
SR_consistency'
%% one tags
clc, clear
one_tags_S1 = [
8.7363	4.4712	7.3851	4.1269	7.1884	11.9767
2.3303	2.3230	1.6318	1.3694	1.9637	2.7427
2.4584	2.8110	2.1117	1.8733	2.2984	3.9435
];

one_tags_S2 = [
3.3213	4.9169	5.2951	4.0811	4.4345	7.7397
2.1368	3.2027	3.2589	2.4480	4.1563	4.4254
2.4965	3.4113	3.3785	2.7518	4.3019	3.6594
];

one_tags_S3 = [
4.9909	9.5620	8.7533	4.6421	10.1308	15.7764
4.6720	5.9350	6.9331	4.4352	8.9493	15.3862
5.2199	5.1830	7.5932	4.4382	9.5586	14.1518
];

one_tags_S4 = [
21.2271	22.1641	17.5779	15.9909	8.8797	15.3636
4.3681	4.7176	4.4804	3.9004	3.3891	3.7440
6.1193	6.3418	5.3863	5.2003	3.5784	5.7562
];

one_tags_S5 = [
3.4621	8.3131	4.8500	7.6217	7.4838	12.4364
2.0590	2.9541	3.0224	3.5483	3.1386	6.0885
1.3295	2.4713	2.5733	1.9232	1.7340	3.7268
];

% one_tags_S6 = [
% 29.4400	27.5404	27.9955	9.7005	20.6511	9.5050
% 5.1207	5.0574	5.3537	2.0539	4.1739	2.4194
% 53.1274	52.7457	52.9046	61.7489	55.5216	59.7329
% ];

one_tags_S7 = [
7.7991	9.9647	7.6857	4.1640	6.1619	2.3398
2.1563	2.7837	3.1058	1.3234	2.4537	2.0838
6.9715	7.5746	7.7603	6.9527	7.6966	6.3605
];

% one_tags_all = [
% one_tags_S1, one_tags_S2, one_tags_S3, one_tags_S4, one_tags_S5, one_tags_S6, one_tags_S7
% ];

one_tags_all = [
one_tags_S1, one_tags_S2, one_tags_S3, one_tags_S4, one_tags_S5, one_tags_S7
];

disp("------------ 2 tags")
mean_one_tags_all = mean(one_tags_all')'
std_one_tags_all = std(one_tags_all')'
one_tags_mean_outperform_ = (mean_one_tags_all(1) - mean_one_tags_all(2))/mean_one_tags_all(1)
one_tags_std_outperform_ = (std_one_tags_all(1) - std_one_tags_all(2))/std_one_tags_all(1)

%% two tags
clc, clear
two_tags_S1 = [
8.7363	4.4712	7.3851	4.1269	7.1884	11.9767
2.3303	2.3230	1.6318	1.3694	1.9637	2.7427
2.4584	2.8110	2.1117	1.8733	2.2984	3.9435
];

two_tags_S2 = [
3.3213	4.9169	5.2951	4.0811	4.4345	7.7397
2.1368	3.2027	3.2589	2.4480	4.1563	4.4254
2.4965	3.4113	3.3785	2.7518	4.3019	3.6594
];

two_tags_S3 = [
4.9909	9.5620	8.7533	4.6421	10.1308	15.7764
4.6720	5.9350	6.9331	4.4352	8.9493	15.3862
5.2199	5.1830	7.5932	4.4382	9.5586	14.1518
];

two_tags_S4 = [
21.2271	22.1641	17.5779	15.9909	8.8797	15.3636
4.3681	4.7176	4.4804	3.9004	3.3891	3.7440
6.1193	6.3418	5.3863	5.2003	3.5784	5.7562
];

two_tags_S5 = [
3.4621	8.3131	4.8500	7.6217	7.4838	12.4364
2.0590	2.9541	3.0224	3.5483	3.1386	6.0885
1.3295	2.4713	2.5733	1.9232	1.7340	3.7268
];

two_tags_S6 = [
29.4400	27.5404	27.9955	9.7005	20.6511	9.5050
5.1207	5.0574	5.3537	2.0539	4.1739	2.4194
5.1274	5.7457	5.9046	6.7489	5.5216	5.7329
];

two_tags_S7 = [
7.7991	9.9647	7.6857	4.1640	6.1619	2.3398
2.1563	2.7837	3.1058	1.3234	2.4537	2.0838
6.9715	7.5746	7.7603	6.9527	7.6966	6.3605
];

two_tags_all = [
two_tags_S1, two_tags_S2, two_tags_S3, two_tags_S4, two_tags_S5, two_tags_S6, two_tags_S7
];

disp("------------ 2 tags")
mean_two_tags_all = mean(two_tags_all')'
std_two_tags_all = std(two_tags_all')'
two_tags_mean_outperform_ = (mean_two_tags_all(1) - mean_two_tags_all(2))/mean_two_tags_all(1)
two_tags_std_outperform_ = (std_two_tags_all(1) - std_two_tags_all(2))/std_two_tags_all(1)

%% four tags
four_tags_S67 = [
6.9426	9.6281	6.9136	3.9650	5.6420
2.2409	2.5607	2.9773	1.8215	2.1995
3.1475	3.0735	3.5983	1.8910	2.7451
];
four_tags_S46 = [
4.2476	8.5054	4.6712	4.3311	2.8748
1.0746	1.8871	2.3619	1.5350	2.6191
1.8631	2.5341	2.5963	2.0956	3.0065
];

four_tags_S56 = [
2.9309	8.0801	4.1196	3.4985	3.2204
1.1541	2.1026	2.5017	1.4267	2.5693
1.0325	2.0526	2.3972	1.4497	2.4457
];

four_tags_S25 = [
3.1991	4.7311	5.4558	4.8344	8.6576
0.9538	2.3810	1.4556	1.1592	2.5277
1.3290	2.4685	2.1663	1.9265	3.8341
];

four_tags_S23 = [
2.8820	4.4672	3.9536	3.7802	6.2324
1.0896	1.5349	1.5938	1.5597	2.7543
1.2629	1.6148	1.7171	1.7248	2.1326
];


four_tags_ALL = [
four_tags_S67, four_tags_S46 four_tags_S56  four_tags_S25 four_tags_S23 
];

disp("------------ 4 tags")
mean_four_tags_all = mean(four_tags_ALL')'
std_four_tags_all = std(four_tags_ALL')'
two_tags_mean_outperform_ = (mean_four_tags_all(1) - mean_four_tags_all(2))/mean_four_tags_all(1)
two_tags_std_outperform_ = (std_four_tags_all(1) - std_four_tags_all(2))/std_four_tags_all(1)

%% 6 tags
six_tags_S456 = [
    3.0474    7.9760    4.1670    3.2358
    1.0347    2.2226    2.5355    2.4769 
];

six_tags_S235 = [
    3.1673    4.2561    3.3700    5.2558
    1.1067    1.9130    1.4411    2.5627    
];

six_tags_S137 = [
    8.1527    3.8816    4.1995    2.4783
    2.0082    1.4406    1.6018    1.7872
];

six_tags_S234 = [
    3.0485    3.9080    2.5171    3.6734
    1.1221    1.7577    1.6302    2.3265
];

six_tags_S123 = [
    4.1963    3.9002    3.3723    5.9765
    1.3497    1.5497    1.4115    2.3588
];

six_tags_S567 = [
    3.0894    8.0879    4.1831    3.5839
    0.9535    1.9322    2.4044    1.3869
];


six_tags_ALL = [
six_tags_S456, six_tags_S235 six_tags_S137   six_tags_S234  six_tags_S123 six_tags_S567
];
disp("------------ 6 tags")

mean_six_tags_all = mean(six_tags_ALL')'
std_six_tags_all = std(six_tags_ALL')'
two_tags_mean_outperform_ = (mean_six_tags_all(1) - mean_six_tags_all(2))/mean_six_tags_all(1)
two_tags_std_outperform_ = (std_six_tags_all(1) - std_six_tags_all(2))/std_six_tags_all(1)


%% eight tags

eight_tags_S1357 = [
    8.0892    3.7280    2.3486
    1.9162    1.5446    1.5322
];

eight_tags_S1257 = [
    4.2222    3.6790    2.3107
    2.4219    1.3490    1.3753
];

eight_tags_S1345 = [
    7.9341    2.5872    3.9556
    2.1287    1.4802    2.1600    
];

eight_tags_S2457 = [
    3.1036    4.1414    2.4012
    1.1756    2.5996    1.7209    
];

% eight_tags_ALL = [
% eight_tags_S1357 eight_tags_S1257  eight_tags_S1345  eight_tags_S2457
% ];

eight_tags_ALL = [
eight_tags_S1357  eight_tags_S1257 eight_tags_S1345 eight_tags_S2457
];

disp("------------ 8 tags")
mean_eight_tags_all = mean(eight_tags_ALL')'
std_eight_tags_all = std(eight_tags_ALL')'
two_tags_mean_outperform_ = (mean_eight_tags_all(1) - mean_eight_tags_all(2))/mean_eight_tags_all(1)
two_tags_std_outperform_ = (std_eight_tags_all(1) - std_eight_tags_all(2))/std_eight_tags_all(1)



%%
all_data = [
two_tags_all, four_tags_ALL, six_tags_ALL, eight_tags_ALL    
];
mean_all = mean(all_data')'
std_all = std(all_data')'
