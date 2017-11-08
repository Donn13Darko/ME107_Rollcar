float prec;

void setup ()
{
  Serial.begin(9600);
}

void loop ()
{
  Serial.print("10^-1: ");
  prec = pow(10, -1);
  Serial.println(prec, 10);
  delay(100);

  Serial.print("10^-2: ");
  prec = pow(10, -2);
  Serial.println(prec, 10);
  delay(100);

  Serial.print("10^-3: ");
  prec = pow(10, -3);
  Serial.println(prec, 10);
  delay(100);

  Serial.print("10^-4: ");
  prec = pow(10, -4);
  Serial.println(prec, 10);
  delay(100);

  Serial.print("10^-5: ");
  prec = pow(10, -5);
  Serial.println(prec, 10);
  delay(100);

  Serial.print("10^-6: ");
  prec = pow(10, -6);
  Serial.println(prec, 10);
  delay(100);

  Serial.print("10^-7: ");
  prec = pow(10, -7);
  Serial.println(prec, 10);
  delay(100);

  Serial.print("10^-8: ");
  prec = pow(10, -8);
  Serial.println(prec, 10);
  delay(100);

  Serial.print("10^-9: ");
  prec = pow(10, -9);
  Serial.println(prec, 10);
  delay(100);

  Serial.print("10^-10: ");
  prec = pow(10, -10);
  Serial.println(prec, 10);
  delay(100);

  Serial.println("");
  Serial.println("");

  delay(10000);
}

