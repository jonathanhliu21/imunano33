#include <gtest/gtest.h>
#include <imunano33/climate.hpp>

using namespace imunano33;

TEST(Climate, NoData) {
  Climate c;
  EXPECT_FALSE(c.dataExists());
}

TEST(Climate, NoDataReset) {
  Climate c;
  c.update(2, 4, 9);
  c.update(3, 0, 4);
  c.update(1, 9, 8);
  c.update(2, 1, 2);
  c.update(4, 2, 4);
  c.update(3, 4, 9);
  c.update(9, 3, 1);
  c.reset();
  EXPECT_FALSE(c.dataExists());
}

TEST(Climate, YesData) {
  Climate c;
  c.update(2, 4, 9);
  c.update(3, 0, 4);
  c.update(1, 9, 8);
  c.update(2, 1, 2);
  c.update(4, 2, 4);
  c.update(3, 4, 9);
  c.update(9, 3, 1);
  EXPECT_TRUE(c.dataExists());
}

TEST(Climate, Copy) {
  Climate c;
  c.update(2, 4, 9);
  c.update(3, 0, 4);
  c.update(1, 9, 8);
  c.update(2, 1, 2);
  c.update(4, 2, 4);
  c.update(3, 4, 9);
  c.update(9, 3, 1);

  Climate c2{c};
  double temp = c2.getTemp<CELSIUS>();
  double humid = c2.getHumidity();
  double pres = c2.getPressure<KPA>();

  EXPECT_NEAR(temp, 9, 0.0001);
  EXPECT_NEAR(humid, 3, 0.0001);
  EXPECT_NEAR(pres, 1, 0.0001);
}

TEST(Climate, Assign) {
  Climate c;
  c.update(2, 4, 9);
  c.update(3, 0, 4);
  c.update(1, 9, 8);
  c.update(2, 1, 2);
  c.update(4, 2, 4);
  c.update(3, 4, 9);
  c.update(9, 3, 1);

  Climate c2;
  c2.update(4, 0, 1);

  c2 = c;

  double temp = c2.getTemp<CELSIUS>();
  double humid = c2.getHumidity();
  double pres = c2.getPressure<KPA>();

  EXPECT_NEAR(temp, 9, 0.0001);
  EXPECT_NEAR(humid, 3, 0.0001);
  EXPECT_NEAR(pres, 1, 0.0001);
}

TEST(Climate, TempF) {
  Climate c;

  c.update(0, 0, 0);
  double res = c.getTemp<FAHRENHEIT>();
  EXPECT_NEAR(res, 32.0, 0.0001);

  c.update(20, 0, 0);
  res = c.getTemp<FAHRENHEIT>();
  EXPECT_NEAR(res, 68.0, 0.0001);

  c.update(46.5, 0, 0);
  res = c.getTemp<FAHRENHEIT>();
  EXPECT_NEAR(res, 115.7, 0.0001);

  c.update(-40.0, 0, 0);
  res = c.getTemp<FAHRENHEIT>();
  EXPECT_NEAR(res, -40.0, 0.0001);
}

TEST(Climate, TempC) {
  Climate c;

  c.update(0, 0, 0);
  double res = c.getTemp<CELSIUS>();
  EXPECT_NEAR(res, 0.0, 0.0001);

  c.update(20, 0, 0);
  res = c.getTemp<CELSIUS>();
  EXPECT_NEAR(res, 20.0, 0.0001);

  c.update(46.5, 0, 0);
  res = c.getTemp<CELSIUS>();
  EXPECT_NEAR(res, 46.5, 0.0001);

  c.update(-40.0, 0, 0);
  res = c.getTemp<CELSIUS>();
  EXPECT_NEAR(res, -40.0, 0.0001);
}

TEST(Climate, TempK) {
  Climate c;

  c.update(0, 0, 0);
  double res = c.getTemp<KELVIN>();
  EXPECT_NEAR(res, 273.15, 0.0001);

  c.update(20, 0, 0);
  res = c.getTemp<KELVIN>();
  EXPECT_NEAR(res, 293.15, 0.0001);

  c.update(46.5, 0, 0);
  res = c.getTemp<KELVIN>();
  EXPECT_NEAR(res, 319.65, 0.0001);

  c.update(-40.0, 0, 0);
  res = c.getTemp<KELVIN>();
  EXPECT_NEAR(res, 233.15, 0.0001);
}

TEST(Climate, PresKPa) {
  Climate c;

  c.update(0, 0, 0);
  double res = c.getPressure<KPA>();
  EXPECT_NEAR(res, 0, 0.0001);

  c.update(0, 0, 20);
  res = c.getPressure<KPA>();
  EXPECT_NEAR(res, 20, 0.0001);

  c.update(0, 0, 46.5);
  res = c.getPressure<KPA>();
  EXPECT_NEAR(res, 46.5, 0.0001);
}

TEST(Climate, PresAtm) {
  Climate c;

  c.update(0, 0, 0);
  double res = c.getPressure<ATM>();
  EXPECT_NEAR(res, 0, 0.0001);

  c.update(0, 0, 20);
  res = c.getPressure<ATM>();
  EXPECT_NEAR(res, 0.197384653, 0.0001);

  c.update(0, 0, 46.5);
  res = c.getPressure<ATM>();
  EXPECT_NEAR(res, 0.458919319, 0.0001);
}

TEST(Climate, PresMmHg) {
  Climate c;

  c.update(0, 0, 0);
  double res = c.getPressure<MMHG>();
  EXPECT_NEAR(res, 0, 0.0001);

  c.update(0, 0, 20);
  res = c.getPressure<MMHG>();
  EXPECT_NEAR(res, 150.012337, 0.0001);

  c.update(0, 0, 46.5);
  res = c.getPressure<MMHG>();
  EXPECT_NEAR(res, 348.778682, 0.0001);
}

TEST(Climate, PresPsi) {
  Climate c;

  c.update(0, 0, 0);
  double res = c.getPressure<PSI>();
  EXPECT_NEAR(res, 0, 0.0001);

  c.update(0, 0, 20);
  res = c.getPressure<PSI>();
  EXPECT_NEAR(res, 2.90075475, 0.0001);

  c.update(0, 0, 46.5);
  res = c.getPressure<PSI>();
  EXPECT_NEAR(res, 6.7442548, 0.0001);
}

TEST(Climate, Humid) {
  Climate c;

  c.update(0, 0, 0);
  double res = c.getHumidity();
  EXPECT_NEAR(res, 0, 0.0001);

  c.update(0, 20, 0);
  res = c.getHumidity();
  EXPECT_NEAR(res, 20, 0.0001);

  c.update(0, 46.5, 0);
  res = c.getHumidity();
  EXPECT_NEAR(res, 46.5, 0.0001);
}
