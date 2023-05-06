// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++  Generate JSON string for key value pair                    +++++
// +++++  Function overloaded to handle both float and int value     +++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
String jsonKeyValue(String key, float value)
{
  String output = "";
  output += "{\"";
  output += key;
  output += "\": ";
  output += value;
  output += "}";
  return output;
}

String jsonKeyValue(String key, int value)
{
  String output = "";
  output += "{\"";
  output += key;
  output += "\": ";
  output += value;
  output += "}";
  return output;
}

String jsonKeyValue(String key, unsigned int value)
{
  String output = "";
  output += "{\"";
  output += key;
  output += "\": ";
  output += value;
  output += "}";
  return output;
}

String jsonKeyValue(String key, String value)
{
  String output = "";
  output += "{\"";
  output += key;
  output += "\": ";
  output += "\"";
  output += value;
  output += "\"";
  output += "}";
  return output;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++  Concatenate two JSON strings, each string must start and end with {, }  +++++
// +++++  if either input string is empty, return the none empty string           +++++
// +++++  if both input strings are empty, empty string will be turned            +++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
String jsonConcat(String json1, String json2)
{
  String output = "";
  String temp1 = json1;
  String temp2 = json2;
  if(temp1 == "")
  {
    return temp2;
  }
  else if(temp2 == "")
  {
    return temp1;
  }
  temp1.remove(temp1.length()-1);  // remove ending } from json1
  temp2.remove(0, 1);  // removing beginning { from json2
  output += temp1;
  output += ", ";
  output += temp2;
  return output; 
}

