#include <stdio.h>
#include <math.h>



// 
// void main()
// {
// 	int i;
// 	for(i = 0; i < 360; i++)
// 	{
// 		float x = 1610.0 + 540 * sin((i + 120) * 2 * M_PI / 360);
// 		printf("%f\n", x);
// 	}
// }
// 


#if 0
int probed[] = 
{
	1235,
	1235,
	1236,
	1238,
	1236,
	1236,
	1238,
	1238,
	1240,
	1239,
	1240,
	1240,
	1241,
	1240,
	1240,
	1241,
	1243,
	1242,
	1242,
	1244,
	1244,
	1244,
	1244,
	1246,
	1246,
	1246,
	1245,
	1248,
	1247,
	1248,
	1250,
	1248,
	1250,
	1250,
	1249,
	1250,
	1250,
	1252,
	1252,
	1252,
	1254,
	1253,
	1253,
	1254,
	1256,
	1254,
	1256,
	1256,
	1257,
	1256,
	1255,
	1257,
	1257,
	1259,
	1258,
	1260,
	1259,
	1260,
	1261,
	1261,
	1261,
	1261,
	1262,
	1263,
	1263,
	1263,
	1265,
	1265,
	1265,
	1265,
	1266,
	1267,
	1268,
	1269,
	1268,
	1267,
	1269,
	1268,
	1269,
	1271,
	1271,
	1271,
	1270,
	1271,
	1274,
	1272,
	1272,
	1275,
	1274,
	1274,
	1276,
	1274,
	1274,
	1275,
	1277,
	1275,
	1277,
	1277,
	1278,
	1278,
	1278,
	1279,
	1282,
	1281,
	1281,
	1281,
	1282,
	1282,
	1283,
	1284,
	1283,
	1283,
	1285,
	1284,
	1286,
	1286,
	1286,
	1285,
	1288,
	1287,
	1290,
	1289,
	1288,
	1289,
	1289,
	1290,
	1289,
	1291,
	1291,
	1291,
	1292,
	1292,
	1293,
	1295,
	1295,
	1294,
	1295,
	1296,
	1296,
	1296,
	1295,
	1296,
	1298,
	1297,
	1299,
	1299,
	1299,
	1299,
	1300,
	1299,
	1301,
	1301,
	1303,
	1302,
	1302,
	1304,
	1303,
	1304,
	1304,
	1305,
	1305,
	1306,
	1305,
	1306,
	1306,
	1308,
	1308,
	1308,
	1309,
	1309,
	1309,
	1310,
	1309,
	1313,
	1310,
	1311,
	1312,
	1313,
	1312,
	1313,
	1313,
	1314,
	1314,
	1314,
	1315,
	1316,
	1316,
	1316,
	1318,
	1317,
	1318,
	1318,
	1318,
	1320,
	1319,
	1321,
	1319,
	1320,
	1321,
	1321,
	1322,
	1322,
	1323,
	1323,
	1323,
	1323,
	1324,
	1325,
	1325,
	1326,
	1326,
	1326,
	1327,
	1328,
	1328,
	1328,
	1329,
	1329,
	1330,
	1329,
	1330,
	1331,
	1332,
	1331,
	1333,
	1333,
	1333,
	1333,
	1334,
	1335,
	1336,
	1335,
	1335,
	1336,
	1336,
	1337,
	1338,
	1337,
	1339,
	1338,
	1339,
	1339,
	1340,
	1340,
	1341,
	1341,
	1340,
	1341,
	1343,
	1342,
	1342,
	1343,
	1345,
	1343,
	1343,
	1345,
	1346,
	1345,
	1347,
	1347,
	1348,
	1346,
	1347,
	1347,
	1349,
	1349,
	1349,
	1350,
	1351,
	1351,
	1351,
	1352,
	1352,
	1352,
	1353,
	1351,
	1354,
	1354,
	1355,
	1355,
	1354,
	1357,
	1355,
	1356,
	1357,
	1357,
	1356,
	1358,
	1360,
	1357,
	1358,
	1359,
	1361,
	1361,
	1362,
	1363,
	1363,
	1362,
	1363,
	1364,
	1363,
	1362,
	1366,
	1366,
	1365,
	1365,
	1366,
	1366,
	1367,
	1366,
	1368,
	1367,
	1369,
	1368,
	1370,
	1370,
	1369,
	1370,
	1370,
	1372,
	1371,
	1372,
	1373,
	1373,
	1373,
	1373,
	1374,
	1373,
	1375,
	1376,
	1375,
	1377,
	1375,
	1375,
	1377,
	1378,
	1377,
	1377,
	1378,
	1379,
	1379,
	1379,
	1379,
	1381,
	1381,
	1381,
	1382,
	1382,
	1382,
	1382,
	1383,
	1383,
	1384,
	1384,
	1384,
	1385,
	1385,
	1386,
	1386,
	1388,
	1387,
	1387,
	1387,
	1388,
	1388,
	1388,
	1389,
	1390,
	1390,
	1390,
	1392,
	1392,
	1392,
	1392,
	1394,
	1393,
	1392,
	1394,
	1395,
	1394,
	1395,
	1397,
	1395,
	1396,
	1396,
	1397,
	1398,
	1398,
	1398,
	1399,
	1398,
	1399,
	1399,
	1401,
	1401,
	1400,
	1401,
	1402,
	1402,
	1402,
	1402,
	1403,
	1404,
	1405,
	1404,
	1405,
	1405,
	1406,
	1406,
	1406,
	1407,
	1407,
	1407,
	1408,
	1408,
	1409,
	1409,
	1410,
	1410,
	1409,
	1410,
	1411,
	1411,
	1411,
	1413,
	1413,
	1412,
	1413,
	1414,
	1413,
	1415,
	1415,
	1416,
	1416,
	1417,
	1417,
	1417,
	1417,
	1417,
	1418,
	1418,
	1419,
	1419,
	1420,
	1420,
	1421,
	1421,
	1420,
	1421,
	1422,
	1422,
	1422,
	1423,
	1423,
	1423,
	1424,
	1424,
	1425,
	1426,
	1425,
	1427,
	1426,
	1426,
	1427,
	1427,
	1427,
	1428,
	1428,
	1429,
	1429,
	1429,
	1430,
	1430,
	1430,
	1431,
	1431,
	1431,
	1432,
	1433,
	1433,
	1433,
	1434,
	1434,
	1434,
	1435,
	1435,
	1436,
	1436,
	1436,
	1437,
	1437,
	1437,
	1438,
	1438,
	1438,
	1438,
	1441,
	1439,
	1439,
	1440,
	1441,
	1440,
	1441,
	1442,
	1443,
	1443,
	1443,
	1444,
	1444,
	1444,
	1445,
	1445,
	1444,
	1446,
	1446,
	1446,
	1447,
	1448,
	1448,
	1447,
	1448,
	1448,
	1449,
	1449,
	1450,
	1451,
	1449,
	1451,
	1451,
	1451,
	1452,
	1452,
	1453,
	1453,
	1453,
	1454,
	1454,
	1455,
	1455,
	1455,
	1456,
	1456,
	1456,
	1456,
	1457,
	1458,
	1458,
	1458,
	1458,
	1459,
	1459,
	1459,
	1459,
	1461,
	1461,
	1461,
	1461,
	1461,
	1462,
	1462,
	1463,
	1463,
	1463,
	1464,
	1464,
	1464,
	1464,
	1466,
	1466,
	1465,
	1466,
	1466,
	1466,
	1466,
	1467,
	1468,
	1468,
	1468,
	1469,
	1470,
	1469,
	1470,
	1470,
	1471,
	1471,
	1471,
	1471,
	1472,
	1473,
	1473,
	1473,
	1474,
	1474,
	1474,
	1474,
	1475,
	1475,
	1476,
	1476,
	1476,
	1476,
	1477,
	1477,
	1478,
	1478,
	1478,
	1478,
	1480,
	1479,
	1480,
	1480,
	1481,
	1480,
	1481,
	1481,
	1482,
	1483,
	1482,
	1483,
	1483,
	1484,
	1484,
	1484,
	1485,
	1485,
	1486,
	1486,
	1486,
	1486,
	1487,
	1487,
	1487,
	1488,
	1488,
	1488,
	1489,
	1489,
	1489,
	1489,
	1490,
	1490,
	1490,
	1491,
	1491,
	1492,
	1492,
	1492,
	1493,
	1493,
	1493,
	1494,
	1494,
	1494,
	1494,
	1495,
	1495,
	1496,
	1496,
	1496,
	1497,
	1497,
	1497,
	1497,
	1498,
	1498,
	1498,
	1499,
	1500,
	1500,
	1500,
	1500,
	1501,
	1501,
	1501,
	1502,
	1503,
	1502,
	1503,
	1504,
	1503,
	1504,
	1504,
	1506,
	1505,
	1505,
	1505,
	1506,
	1506,
	1506,
	1506,
	1507,
	1507,
	1508,
	1508,
	1508,
	1509,
	1509,
	1509,
	1510,
	1510,
	1510,
	1511,
	1511,
	1512,
	1512,
	1512,
	1512,
	1512,
	1512,
	1513,
	1514,
	1514,
	1514,
	1514,
	1514,
	1515,
	1515,
	1516,
	1516,
	1516,
	1517,
	1517,
	1517,
	1517,
	1518,
	1518,
	1519,
	1519,
	1519,
	1520,
	1520,
	1521,
	1521,
	1521,
	1521,
	1521,
	1522,
	1522,
	1522,
	1523,
	1524,
	1523,
	1524,
	1524,
	1524,
	1525,
	1525,
	1525,
	1525,
	1525,
	1526,
	1527,
	1527,
	1528,
	1528,
	1528,
	1528,
	1528,
	1529,
	1529,
	1529,
	1530,
	1530,
	1530,
	1530,
	1531,
	1531,
	1531,
	1532,
	1532,
	1532,
	1533,
	1532,
	1533,
	1533,
	1534,
	1535,
	1534,
	1535,
	1535,
	1535,
	1535,
	1536,
	1536,
	1537,
	1537,
	1537,
	1538,
	1538,
	1538,
	1538,
	1538,
	1539,
	1540,
	1540,
	1540,
	1540,
	1541,
	1541,
	1540,
	1542,
	1542,
	1543,
	1542,
	1543,
	1543,
	1543,
	1543,
	1544,
	1544,
	1544,
	1545,
	1545,
	1545,
	1545,
	1547,
	1546,
	1546,
	1547,
	1547,
	1547,
	1547,
	1547,
	1547,
	1548,
	1548,
	1549,
	1549,
	1549,
	1549,
	1550,
	1550,
	1551,
	1551,
	1551,
	1551,
	1552,
	1552,
	1552,
	1552,
	1553,
	1553,
	1553,
	1553,
	1553,
	1554,
	1554,
	1555,
	1555,
	1555,
	1556,
	1556,
	1556,
	1556,
	1556,
	1558,
	1557,
	1557,
	1558,
	1558,
	1558,
	1558,
	1558,
	1559,
	1560,
	1559,
	1559,
	1560,
	1561,
	1560,
	1561,
	1560,
	1561,
	1561,
	1561,
	1562,
	1562,
	1563,
	1562,
	1563,
	1563,
	1563,
	1563,
	1563,
	1564,
	1564,
	1563,
	1564,
	1564,
	1565,
	1564,
	1564,
	1564,
	1564,
	1564,
	1564,
	1565,
	1565,
	1565,
	1565,
	1565,
	1566,
	1565,
	1565,
	1565,
	1565,
	1565,
	1566,
	1566,
	1566,
	1566,
	1566,
	1566,
	1566,
	1567,
	1566,
	1567,
	1566,
	1567,
	1567,
	1567,
	1567,
	1567,
	1567,
	1567,
	1567,
	1567,
	1567,
	1567,
	1568,
	1568,
	1568,
	1568,
	1569,
	1568,
	1568,
	1568,
	1568,
	1568,
	1568,
	1568,
	1569,
	1569,
	1569,
	1569,
	1569,
	1569,
	1569,
	1569,
	1569,
	1569,
	1569,
	1570,
	1569,
	1571,
	1570,
	1570,
	1570,
	1570,
	1570,
	1570,
	1570,
	1571,
	1571,
	1571,
	1571,
	1571,
	1571,
	1571,
	1571,
	1571,
	1571,
	1571,
	1571,
	1572,
	1572,
	1572,
	1572,
	1572,
	1572,
	1572,
	1572,
	1573,
	1572,
	1572,
	1573,
	1572,
	1572,
	1573,
	1573,
	1573,
	1574,
	1573,
	1573,
	1573,
	1574,
	1573,
	1574,
	1573,
	1574,
	1574,
	1574,
	1574,
	1575,
	1574,
	1574,
	1574,
	1574,
	1574,
	1575,
	1575,
	1575,
	1575,
	1575,
	1575,
	1575,
	1575,
	1575,
	1575,
	1575,
	1576,
	1576,
	1576,
	1576,
	1576,
	1576,
	1576,
	1576,
	1576,
	1576,
	1576,
	1576,
	1576,
	1577,
	1577,
	1577,
	1578,
	1577,
	1577,
	1577,
	1577,
	1577,
	1577,
	1577,
	1578,
	1578,
	1578,
	1578,
	1578,
	1578,
	1578,
	1578,
	1578,
	1578,
	1578,
	1578,
	1578,
	1578,
	1579,
	1579,
	1579,
	1579,
	1579,
	1579,
	1579,
	1579,
	1579,
	1579,
	1580,
	1580,
	1580,
	1580,
	1580,
	1580,
	1580,
	1580,
	1580,
	1580,
	1580,
	1580,
	1580,
	1580,
	1581,
	1581,
	1581,
	1581,
	1581,
	1581,
	1581,
	1581,
	1581,
	1581,
	1582,
	1582,
	1581,
	1582,
	1582,
	1582,
	1582,
	1582,
	1582,
	1582,
	1582,
	1582,
	1582,
	1582,
	1583,
	1582,
	1583,
	1583,
	1583,
	1583,
	1583,
	1583,
	1583,
	1583,
	1583,
	1583,
	1584,
	1584,
	1584,
	1584,
	1584,
	1584,
	1584,
	1584,
	1584,
	1584,
	1584,
	1584,
	1584,
	1585,
	1585,
	1585,
	1585,
	1585,
	1585,
	1585,
	1585,
	1585,
	1585,
	1585,
	1586,
	1586,
	1586,
	1586,
	1586,
	1586,
	1587,
	1586,
	1586,
	1586,
	1586,
	1586,
	1586,
	1587,
	1587,
	1586,
	1587,
	1587,
	1587,
	1587,
	1587,
	1587,
	1587,
	1587,
	1588,
	1587,
	1587,
	1588,
	1588,
	1588,
	1588,
	1588,
	1588,
	1588,
	1588,
	1588,
	1588,
	1588,
	1589,
	1589,
	1589,
	1589,
	1589,
	1589,
	1589,
	1589,
	1589,
	1589,
	1590,
	1589,
	1590,
	1590,
	1590,
	1590,
	1590,
	1590,
	1590,
	1590,
	1590,
	1590,
	1590,
	1590,
	1591,
	1591,
	1590,
	1591,
	1591,
	1591,
	1591,
	1591,
	1591,
	1591,
	1591,
	1591,
	1591,
	1592,
	1592,
	1592,
	1592,
	1592,
	1592,
	1592,
	1592,
	1592,
	1592,
	1592,
	1593,
	1592,
	1593,
	1593,
	1592,
	1593,
	1593,
	1593,
	1593,
	1593,
	1593,
	1593,
	1593,
	1593,
	1593,
	1593,
	1593,
	1594,
	1594,
	1594,
	1594,
	1594,
	1594,
	1594,
	1594,
	1594,
	1594,
	1594,
	1594,
	1594,
	1595,
	1595,
	1595,
	1595,
	1595,
	1595,
	1595,
	1595,
	1595,
	1595,
	1595,
	1595,
	1595,
	1595,
	1596,
	1596,
	1596,
	1596,
	1596,
	1596,
	1596,
	1596,
	1596,
	1596,
	1596,
	1596,
	1597,
	1597,
	1597,
	1597,
	1597,
	1597,
	1597,
	1597,
	1597,
	1597,
	1597,
	1597,
	1597,
	1597,
	1598,
	1598,
	1598,
	1598,
	1598,
	1598,
	1598,
	1598,
	1598,
	1598,
	1598,
	1598,
	1598,
	1598,
	1599,
	1599,
	1599,
	1599,
	1600,
	1599,
	1599,
	1599,
	1599,
	1599,
	1599,
	1600,
	1600,
	1599,
	1600,
	1600,
	1600,
	1600,
	1600,
	1600,
	1600,
	1601,
	1600,
	1600,
	1600,
	1600,
	1600,
	1600,
	1601,
	1601,
	1601,
	1601,
	1601,
	1601,
	1601,
	1601,
	1601,
	1601,
	1601,
	1601,
	1602,
	1602,
	1602,
	1602,
	1602,
	1602,
	1602,
	1602,
	1602,
	1602,
	1602,
	1602,
	1602,
	1602,
	1602,
	1603,
	1603,
	1602,
	1603,
	1603,
	1603,
	1604,
	1603,
	1603,
	1603,
	1603,
	1603,
	1603,
	1603,
	1604,
	1604,
	1604,
	1604,
	1604,
	1604,
	1604,
	1604,
	1604,
	1604,
	1605,
	1604,
	1604,
	1604,
	1605,
	1605,
	1604,
	1605,
	1605,
	1605,
	1605,
	1605,
	1605,
	1605,
	1605,
	1605,
	1605,
	1605,
	1605,
	1606,
	1606,
	1606,
	1606,
	1606,
	1606,
	1606,
	1606,
	1606,
	1606,
	1606,
	1606,
	1607,
	1607,
	1606,
	1607,
	1607,
	1607,
	1607,
	1607,
	1608,
	1607,
	1607,
	1607,
	1607,
	1607,
	1607,
	1607,
	1608,
	1607,
	1608,
	1608,
	1608,
	1608,
	1608,
	1608,
	1608,
	1608,
	1608,
	1608,
	1608,
	1608,
	1608,
	1608,
	1608,
	1608,
	1608,
	1608,
	1608,
	1609,
	1609,
	1609,
	1609,
	1609,
	1609,
	1609,
	1609,
	1609,
	1609,
	1609,
	1609,
	1609,
	1609,
	1609,
	1609,
	1609,
	1609,
	1609,
	1610,
	1610,
	1609,
	1610,
	1610,
	1610,
	1610,
	1610,
	1611,
	1610,
	1610,
	1610,
	1610,
	1610,
	1610,
	1610,
	1610,
	1610,
	1610,
	1610,
	1610,
	1610,
	1611,
	1610,
	1610,
	1610,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1611,
	1612,
	1612,
	1612,
	1611,
	1611,
	1612,
	1611,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1611,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1613,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1613,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612,
	1612
};

#define MAX_PWM 1800

void main()
{
	int i, j;
	int probes = sizeof(probed) / sizeof(int);
	for(i = 0; i < MAX_PWM; i++)
	{
		int want = i * (probed[probes - 1] - probed[0]) / MAX_PWM + probed[0];
	
		int min_diff = 0x7fffffff;
		int min_j = -1;
		for(j = 0; j < probes; j++)
		{
			if(abs(probed[j] - want) < min_diff)
			{
				min_j = j;
				min_diff = abs(probed[j] - want);
			}
		}
		
		
		int translated = 0;
		if(i < 1580)
		{
			translated = i * 900 / 1580;
		}
		else
		{
			translated = (i - 1580) * (1800 - 900) / (1900 - 1580) + 900;
		}
		
		
		printf("%d %d\n", min_j, translated);
	}
}

#endif // 0


int probed1[] = 
{
	1614,
	1615,
	1618,
	1622,
	1623,
	1624,
	1624,
	1625,
	1626,
	1630,
	1639,
	1643,
	1651,
	1655,
	1659,
	1666,
	1669,
	1676,
	1679,
	1685,
	1687,
	1690,
	1694,
	1697,
	1702,
	1703,
	1705,
	1709,
	1709,
	1712,
	1714,
	1716,
	1717,
	1718,
	1719,
	1720,
	1721,
	1721,
	1720,
	1721,
	1720,
	1720,
	1720,
	1720,
	1719,
	1716,
	1716,
	1715,
	1714,
	1713,
	1710,
	1711,
	1710,
	1707,
	1705,
	1702,
	1699,
	1696,
	1694,
	1691,
	1689,
	1689,
	1686,
	1686,
	1682,
	1679,
	1679,
	1674,
	1673,
	1672,
	1668,
	1670,
	1667,
	1666,
	1664,
	1664,
	1662,
	1662,
	1661,
	1659,
	1660,
	1657,
	1657,
	1656,
	1656,
	1656,
	1653,
	1654,
	1656,
	1654,
	1654,
	1655,
	1653,
	1656,
	1656,
	1656,
	1654,
	1656,
	1659,
	1656,
	1659,
	1659,
	1660,
	1659,
	1662,
	1663,
	1664,
	1665,
	1666,
	1668,
	1669,
	1669,
	1671,
	1674,
	1675,
	1677,
	1679,
	1682,
	1684,
	1686,
	1689,
	1688,
	1693,
	1693,
	1697,
	1698,
	1703,
	1703,
	1706,
	1710,
	1711,
	1712,
	1713,
	1715,
	1713,
	1716,
	1716,
	1718,
	1718,
	1718,
	1720,
	1719,
	1720,
	1717,
	1719,
	1718,
	1719,
	1716,
	1716,
	1715,
	1713,
	1712,
	1710,
	1707,
	1704,
	1703,
	1701,
	1698,
	1695,
	1691,
	1689,
	1686,
	1680,
	1677,
	1672,
	1668,
	1661,
	1658,
	1654,
	1646,
	1642,
	1634,
	1630,
	1625,
	1625,
	1624,
	1624,
	1623,
	1622,
	1621,
	1616,
	1612,
	1607,
	1600,
	1599,
	1598,
	1598,
	1598,
	1598,
	1597,
	1588,
	1583,
	1572,
	1566,
	1560,
	1548,
	1542,
	1530,
	1523,
	1510,
	1504,
	1496,
	1483,
	1476,
	1461,
	1454,
	1446,
	1432,
	1424,
	1409,
	1402,
	1385,
	1379,
	1370,
	1354,
	1347,
	1331,
	1322,
	1308,
	1300,
	1291,
	1276,
	1268,
	1254,
	1245,
	1231,
	1223,
	1217,
	1200,
	1191,
	1181,
	1173,
	1168,
	1157,
	1155,
	1150,
	1148,
	1144,
	1141,
	1137,
	1125,
	1120,
	1112,
	1109,
	1105,
	1105,
	1101,
	1103,
	1099,
	1093,
	1089,
	1083,
	1076,
	1072,
	1065,
	1060,
	1055,
	1051,
	1047,
	1042,
	1039,
	1035,
	1033,
	1030,
	1027,
	1026,
	1024,
	1023,
	1022,
	1021,
	1022,
	1022,
	1022,
	1022,
	1023,
	1024,
	1026,
	1027,
	1030,
	1033,
	1036,
	1039,
	1043,
	1047,
	1050,
	1056,
	1061,
	1066,
	1073,
	1079,
	1082,
	1087,
	1096,
	1100,
	1104,
	1105,
	1107,
	1109,
	1110,
	1116,
	1119,
	1129,
	1136,
	1141,
	1147,
	1148,
	1155,
	1156,
	1162,
	1166,
	1173,
	1186,
	1192,
	1208,
	1216,
	1230,
	1237,
	1246,
	1260,
	1268,
	1284,
	1291,
	1300,
	1315,
	1324,
	1337,
	1347,
	1362,
	1370,
	1377,
	1393,
	1402,
	1416,
	1425,
	1439,
	1448,
	1455,
	1468,
	1476,
	1490,
	1497,
	1504,
	1517,
	1524,
	1536,
	1542,
	1555,
	1560,
	1566,
	1578,
	1583,
	1593,
	1597,
	1598,
	1597,
	1598,
	1598,
	1598,
	1603,
	1607,
	1616
};

int probed2[] = 
{
	1709,
	1718,
	1719,
	1721,
	1722,
	1726,
	1729,
	1728,
	1732,
	1734,
	1737,
	1737,
	1741,
	1740,
	1742,
	1739,
	1743,
	1747,
	1743,
	1745,
	1745,
	1746,
	1747,
	1747,
	1746,
	1746,
	1743,
	1745,
	1743,
	1741,
	1739,
	1739,
	1739,
	1735,
	1734,
	1732,
	1729,
	1725,
	1722,
	1719,
	1717,
	1711,
	1709,
	1703,
	1700,
	1694,
	1690,
	1687,
	1679,
	1676,
	1668,
	1663,
	1659,
	1651,
	1650,
	1649,
	1649,
	1649,
	1648,
	1647,
	1645,
	1641,
	1633,
	1628,
	1625,
	1625,
	1624,
	1624,
	1624,
	1623,
	1619,
	1613,
	1602,
	1597,
	1585,
	1579,
	1567,
	1562,
	1555,
	1541,
	1535,
	1522,
	1514,
	1500,
	1494,
	1486,
	1471,
	1464,
	1450,
	1442,
	1426,
	1419,
	1411,
	1396,
	1387,
	1372,
	1365,
	1357,
	1340,
	1332,
	1319,
	1308,
	1295,
	1287,
	1279,
	1264,
	1257,
	1243,
	1235,
	1221,
	1211,
	1207,
	1193,
	1188,
	1184,
	1181,
	1178,
	1173,
	1172,
	1162,
	1157,
	1145,
	1142,
	1140,
	1138,
	1136,
	1131,
	1132,
	1130,
	1125,
	1119,
	1112,
	1107,
	1099,
	1095,
	1088,
	1087,
	1081,
	1075,
	1073,
	1070,
	1066,
	1063,
	1059,
	1059,
	1054,
	1056,
	1053,
	1052,
	1052,
	1052,
	1052,
	1051,
	1053,
	1052,
	1054,
	1055,
	1058,
	1060,
	1064,
	1066,
	1068,
	1072,
	1075,
	1081,
	1086,
	1091,
	1096,
	1100,
	1107,
	1109,
	1120,
	1124,
	1129,
	1131,
	1131,
	1136,
	1135,
	1140,
	1142,
	1150,
	1155,
	1161,
	1173,
	1173,
	1178,
	1182,
	1185,
	1189,
	1192,
	1205,
	1211,
	1225,
	1232,
	1243,
	1253,
	1264,
	1278,
	1284,
	1300,
	1309,
	1317,
	1330,
	1340,
	1355,
	1363,
	1369,
	1386,
	1394,
	1410,
	1417,
	1432,
	1440,
	1449,
	1463,
	1470,
	1485,
	1491,
	1506,
	1514,
	1519,
	1533,
	1541,
	1554,
	1559,
	1572,
	1578,
	1584,
	1595,
	1601,
	1612,
	1617,
	1622,
	1624,
	1624,
	1625,
	1625,
	1626,
	1627,
	1631,
	1640,
	1644,
	1649,
	1649,
	1651,
	1651,
	1651,
	1652,
	1653,
	1658,
	1662,
	1666,
	1674,
	1679,
	1685,
	1689,
	1696,
	1700,
	1702,
	1707,
	1711,
	1716,
	1718,
	1724,
	1726,
	1727,
	1731,
	1731,
	1734,
	1737,
	1739,
	1740,
	1740,
	1741,
	1745,
	1743,
	1745,
	1745,
	1745,
	1747,
	1745,
	1746,
	1744,
	1745,
	1743,
	1744,
	1742,
	1740,
	1739,
	1738,
	1735,
	1738,
	1733,
	1735,
	1731,
	1727,
	1727,
	1721,
	1721,
	1718,
	1715,
	1714,
	1710,
	1713,
	1709,
	1708,
	1704,
	1701,
	1699,
	1697,
	1698,
	1695,
	1692,
	1692,
	1691,
	1688,
	1688,
	1688,
	1686,
	1685,
	1684,
	1684,
	1684,
	1682,
	1682,
	1680,
	1682,
	1680,
	1681,
	1680,
	1680,
	1679,
	1682,
	1680,
	1680,
	1679,
	1681,
	1683,
	1682,
	1685,
	1682,
	1684,
	1685,
	1686,
	1685,
	1687,
	1690,
	1690,
	1692,
	1692,
	1695,
	1694,
	1697,
	1700,
	1700,
	1700,
	1704,
	1709,
	1710,
	1711,
	1713
};





void main()
{
	int i, j;
//	for(i = 0; i < 360; i++)
//	{
// current 2
//		float want2 = 1635.0 + 580 * sin((i + 120) * 2 * M_PI / 360);
// current 1
//		float want1 = 1610.0 + 585 * sin((i + 0) * 2 * M_PI / 360);
//		printf("%f\n", want1);
//	}

	for(i = 0; i < 90; i++)
	{
// current 2
		int degrees = i + 180;
		int want2 = (int)(1635.0 + 580 * sin((degrees + 120) * 2 * M_PI / 360));
// current 1
		int want1 = (int)(1610.0 + 585 * sin((degrees + 0) * 2 * M_PI / 360));
		
		int best_j = -1;
		int best_diff = 0x7fffffff;
		for(j = 180; j < 270; j++)
		{
			int diff = abs(want1 - probed1[j]);
			if(diff < best_diff)
			{
				best_diff = diff;
				best_j = j;
			}
		}

		printf("%d %d %d %d %d %d\n", 
			degrees, 
			best_j, 
			want1, 
			probed1[best_j], 
			want2, 
			probed2[best_j]);
		
		
	}
}












