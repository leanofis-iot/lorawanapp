<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.3.0">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="SamacSys_Parts">
<description>&lt;b&gt;https://componentsearchengine.com&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by SamacSys&lt;/author&gt;</description>
<packages>
<package name="LED-RGB-3528-PLCC4">
<description>&lt;b&gt;3.5x2.8mm_Surface_Mount_LED_LAMP&lt;/b&gt;&lt;br&gt;
</description>
<smd name="3" x="1.4" y="-0.725" dx="1.2" dy="0.9" layer="1"/>
<smd name="2" x="-1.4" y="-0.725" dx="1.2" dy="0.9" layer="1"/>
<smd name="1" x="-1.4" y="0.725" dx="1.2" dy="0.9" layer="1"/>
<smd name="4" x="1.4" y="0.725" dx="1.2" dy="0.9" layer="1"/>
<text x="0.888340625" y="2.6047" size="1.27" layer="25" align="center">&gt;NAME</text>
<wire x1="-1.6" y1="1.4" x2="1.6" y2="1.4" width="0.1" layer="51"/>
<wire x1="1.6" y1="1.4" x2="1.6" y2="-1.4" width="0.1" layer="51"/>
<wire x1="1.6" y1="-1.4" x2="-1.6" y2="-1.4" width="0.1" layer="51"/>
<wire x1="-1.6" y1="-1.4" x2="-1.6" y2="1.4" width="0.1" layer="51"/>
<circle x="-0.5" y="0.747" radius="0.071" width="0.2" layer="21"/>
<wire x1="-1.6" y1="1.4" x2="1.6" y2="1.4" width="0.2" layer="21"/>
<wire x1="-1.6" y1="-1.4" x2="1.6" y2="-1.4" width="0.2" layer="21"/>
</package>
<package name="0603">
<description>&lt;b&gt;ERA3A&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-0.8" y="0" dx="1" dy="0.95" layer="1" rot="R90"/>
<smd name="2" x="0.8" y="0" dx="1" dy="0.95" layer="1" rot="R90"/>
<text x="0" y="1.27" size="1.27" layer="25" align="center">&gt;NAME</text>
<wire x1="-0.8" y1="0.4" x2="0.8" y2="0.4" width="0.1" layer="51"/>
<wire x1="0.8" y1="0.4" x2="0.8" y2="-0.4" width="0.1" layer="51"/>
<wire x1="0.8" y1="-0.4" x2="-0.8" y2="-0.4" width="0.1" layer="51"/>
<wire x1="-0.8" y1="-0.4" x2="-0.8" y2="0.4" width="0.1" layer="51"/>
<wire x1="-0.15" y1="0.4" x2="0.15" y2="0.4" width="0.2" layer="21"/>
<wire x1="0.15" y1="0.4" x2="0.15" y2="-0.4" width="0.2" layer="21"/>
<wire x1="0.15" y1="-0.4" x2="-0.15" y2="-0.4" width="0.2" layer="21"/>
<wire x1="-0.15" y1="-0.4" x2="-0.15" y2="0.4" width="0.2" layer="21"/>
</package>
<package name="ESP32-WROOM-32D">
<description>&lt;b&gt;ESP32-WROOM-32D-1&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-8.53655" y="8.2921" dx="2" dy="0.9" layer="1"/>
<smd name="2" x="-8.53655" y="7.0221" dx="2" dy="0.9" layer="1"/>
<smd name="3" x="-8.53655" y="5.7521" dx="2" dy="0.9" layer="1"/>
<smd name="4" x="-8.53655" y="4.4821" dx="2" dy="0.9" layer="1"/>
<smd name="5" x="-8.53655" y="3.2121" dx="2" dy="0.9" layer="1"/>
<smd name="6" x="-8.53655" y="1.9421" dx="2" dy="0.9" layer="1"/>
<smd name="7" x="-8.53655" y="0.6721" dx="2" dy="0.9" layer="1"/>
<smd name="8" x="-8.53655" y="-0.5979" dx="2" dy="0.9" layer="1"/>
<smd name="9" x="-8.53655" y="-1.8679" dx="2" dy="0.9" layer="1"/>
<smd name="10" x="-8.53655" y="-3.1379" dx="2" dy="0.9" layer="1"/>
<smd name="11" x="-8.53655" y="-4.4079" dx="2" dy="0.9" layer="1"/>
<smd name="12" x="-8.53655" y="-5.6779" dx="2" dy="0.9" layer="1"/>
<smd name="13" x="-8.53655" y="-6.9479" dx="2" dy="0.9" layer="1"/>
<smd name="14" x="-8.53655" y="-8.2179" dx="2" dy="0.9" layer="1"/>
<smd name="15" x="-5.75155" y="-9.2179" dx="2" dy="0.9" layer="1" rot="R90"/>
<smd name="16" x="-4.48155" y="-9.2179" dx="2" dy="0.9" layer="1" rot="R90"/>
<smd name="17" x="-3.21155" y="-9.2179" dx="2" dy="0.9" layer="1" rot="R90"/>
<smd name="18" x="-1.94155" y="-9.2179" dx="2" dy="0.9" layer="1" rot="R90"/>
<smd name="19" x="-0.67155" y="-9.2179" dx="2" dy="0.9" layer="1" rot="R90"/>
<smd name="20" x="0.59845" y="-9.2179" dx="2" dy="0.9" layer="1" rot="R90"/>
<smd name="21" x="1.86845" y="-9.2179" dx="2" dy="0.9" layer="1" rot="R90"/>
<smd name="22" x="3.13845" y="-9.2179" dx="2" dy="0.9" layer="1" rot="R90"/>
<smd name="23" x="4.40845" y="-9.2179" dx="2" dy="0.9" layer="1" rot="R90"/>
<smd name="24" x="5.67845" y="-9.2179" dx="2" dy="0.9" layer="1" rot="R90"/>
<smd name="25" x="8.46345" y="-8.2179" dx="2" dy="0.9" layer="1"/>
<smd name="26" x="8.46345" y="-6.9479" dx="2" dy="0.9" layer="1"/>
<smd name="27" x="8.46345" y="-5.6779" dx="2" dy="0.9" layer="1"/>
<smd name="28" x="8.46345" y="-4.4079" dx="2" dy="0.9" layer="1"/>
<smd name="29" x="8.46345" y="-3.1379" dx="2" dy="0.9" layer="1"/>
<smd name="30" x="8.46345" y="-1.8679" dx="2" dy="0.9" layer="1"/>
<smd name="31" x="8.46345" y="-0.5979" dx="2" dy="0.9" layer="1"/>
<smd name="32" x="8.46345" y="0.6721" dx="2" dy="0.9" layer="1"/>
<smd name="33" x="8.46345" y="1.9421" dx="2" dy="0.9" layer="1"/>
<smd name="34" x="8.46345" y="3.2121" dx="2" dy="0.9" layer="1"/>
<smd name="35" x="8.46345" y="4.4821" dx="2" dy="0.9" layer="1"/>
<smd name="36" x="8.46345" y="5.7521" dx="2" dy="0.9" layer="1"/>
<smd name="37" x="8.46345" y="7.0221" dx="2" dy="0.9" layer="1"/>
<smd name="38" x="8.46345" y="8.2921" dx="2" dy="0.9" layer="1"/>
<smd name="39" x="-1.03655" y="0.7921" dx="5" dy="5" layer="1"/>
<wire x1="-9.03655" y1="15.7821" x2="8.96345" y2="15.7821" width="0.1" layer="51"/>
<wire x1="8.96345" y1="15.7821" x2="8.96345" y2="-9.7179" width="0.1" layer="51"/>
<wire x1="8.96345" y1="-9.7179" x2="-9.03655" y2="-9.7179" width="0.1" layer="51"/>
<wire x1="-9.03655" y1="-9.7179" x2="-9.03655" y2="15.7821" width="0.1" layer="51"/>
<wire x1="-9.03655" y1="-9.2179" x2="-9.03655" y2="-9.7179" width="0.2" layer="21"/>
<wire x1="-9.03655" y1="-9.7179" x2="-7.03655" y2="-9.7179" width="0.2" layer="21"/>
<wire x1="6.96345" y1="-9.7179" x2="8.96345" y2="-9.7179" width="0.2" layer="21"/>
<wire x1="8.96345" y1="-9.7179" x2="8.96345" y2="-9.2179" width="0.2" layer="21"/>
<wire x1="-9.03655" y1="9.5321" x2="-9.03655" y2="15.7821" width="0.2" layer="21"/>
<wire x1="-9.03655" y1="15.7821" x2="8.96345" y2="15.7821" width="0.2" layer="21"/>
<wire x1="8.96345" y1="15.7821" x2="8.96345" y2="9.5321" width="0.2" layer="21"/>
<text x="-2.92655" y="6.5921" size="1.27" layer="21">&gt;NAME</text>
<text x="-2.92655" y="4.0521" size="1.27" layer="21">&gt;VALUE</text>
<circle x="-6.63655" y="8.3321" radius="0.4242625" width="0" layer="21"/>
</package>
<package name="HEADER-5P-DıP-2_54MM">
<description>&lt;b&gt;825433-5&lt;/b&gt;&lt;br&gt;
</description>
<pad name="1" x="0" y="0" drill="1.1" diameter="1.65"/>
<pad name="2" x="2.54" y="0" drill="1.1" diameter="1.65"/>
<pad name="3" x="5.08" y="0" drill="1.1" diameter="1.65"/>
<pad name="4" x="7.62" y="0" drill="1.1" diameter="1.65"/>
<pad name="5" x="10.16" y="0" drill="1.1" diameter="1.65"/>
<text x="1.27" y="2.54" size="1.27" layer="25" align="center">&gt;NAME</text>
<wire x1="-1.52" y1="1.27" x2="11.68" y2="1.27" width="0.1" layer="51"/>
<wire x1="11.68" y1="1.27" x2="11.68" y2="-1.27" width="0.1" layer="51"/>
<wire x1="11.68" y1="-1.27" x2="-1.52" y2="-1.27" width="0.1" layer="51"/>
<wire x1="-1.52" y1="-1.27" x2="11.68" y2="-1.27" width="0.2" layer="21"/>
<wire x1="11.68" y1="-1.27" x2="11.68" y2="1.27" width="0.2" layer="21"/>
<wire x1="-1.524" y1="1.27" x2="11.684" y2="1.27" width="0.2" layer="21"/>
<wire x1="-1.524" y1="1.27" x2="-1.524" y2="-1.27" width="0.1" layer="51"/>
<wire x1="-1.524" y1="1.27" x2="-1.524" y2="-1.27" width="0.2" layer="21"/>
<circle x="-1.1" y="0.8" radius="0.25" width="0" layer="21"/>
</package>
<package name="BUTTON-4P-SMD-5X5">
<description>&lt;b&gt;COM-08720-1&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-3.1" y="1.85" dx="1.8" dy="1.1" layer="1"/>
<smd name="4" x="3.1" y="1.85" dx="1.8" dy="1.1" layer="1"/>
<smd name="2" x="-3.1" y="-1.85" dx="1.8" dy="1.1" layer="1"/>
<smd name="3" x="3.1" y="-1.85" dx="1.8" dy="1.1" layer="1"/>
<text x="-0.5" y="3.8" size="1.27" layer="25" align="center">&gt;NAME</text>
<wire x1="-2.6" y1="1" x2="-2.6" y2="-1" width="0.1" layer="51"/>
<wire x1="-2.6" y1="-1" x2="-1" y2="-2.6" width="0.1" layer="51"/>
<wire x1="-1" y1="-2.6" x2="1" y2="-2.6" width="0.1" layer="51"/>
<wire x1="1" y1="-2.6" x2="2.6" y2="-1" width="0.1" layer="51"/>
<wire x1="2.6" y1="-1" x2="2.6" y2="1" width="0.2" layer="51"/>
<wire x1="2.6" y1="1" x2="1" y2="2.6" width="0.1" layer="51"/>
<wire x1="1" y1="2.6" x2="-1" y2="2.6" width="0.2" layer="51"/>
<wire x1="-1" y1="2.6" x2="-2.6" y2="1" width="0.1" layer="51"/>
<wire x1="-2.6" y1="1" x2="-2.6" y2="-1" width="0.2" layer="21"/>
<wire x1="-1" y1="-2.6" x2="1" y2="-2.6" width="0.2" layer="21"/>
<wire x1="2.6" y1="-1" x2="2.6" y2="1" width="0.1" layer="21"/>
<wire x1="-1" y1="2.6" x2="1" y2="2.6" width="0.1" layer="21"/>
<wire x1="1" y1="-2.6" x2="1.8" y2="-1.8" width="0.2" layer="21"/>
<wire x1="1" y1="2.6" x2="1.8" y2="1.8" width="0.2" layer="21"/>
<wire x1="-1" y1="2.6" x2="-1.8" y2="1.8" width="0.2" layer="21"/>
<wire x1="-1" y1="-2.6" x2="-1.85" y2="-1.75" width="0.2" layer="21"/>
</package>
<package name="BH-411-1P24">
<description>&lt;b&gt;825433-5&lt;/b&gt;&lt;br&gt;
</description>
<pad name="1" x="0" y="0" drill="1.1" diameter="2"/>
<pad name="2" x="44.9" y="0" drill="1.1" diameter="2"/>
<text x="1.82" y="7.89" size="1.27" layer="25" align="center">&gt;NAME</text>
<wire x1="-2.55" y1="-6.5" x2="-2.55" y2="6.5" width="0.1" layer="51"/>
<wire x1="-2.55" y1="6.5" x2="47.45" y2="6.5" width="0.1" layer="51"/>
<wire x1="-2.55" y1="6.5" x2="47.45" y2="6.5" width="0.2" layer="21"/>
<wire x1="47.45" y1="6.5" x2="47.45" y2="-6.5" width="0.2" layer="21"/>
<wire x1="47.45" y1="-6.5" x2="-2.55" y2="-6.5" width="0.2" layer="21"/>
<wire x1="-2.55" y1="-6.5" x2="-2.55" y2="6.5" width="0.2" layer="21"/>
<wire x1="4" y1="2" x2="4" y2="-2" width="0.2" layer="21"/>
<wire x1="2" y1="0" x2="6" y2="0" width="0.2" layer="21"/>
<wire x1="42" y1="0" x2="38" y2="0" width="0.2" layer="21"/>
</package>
</packages>
<symbols>
<symbol name="LED-RGB-A">
<wire x1="5.08" y1="2.54" x2="15.24" y2="2.54" width="0.254" layer="94"/>
<wire x1="15.24" y1="-5.08" x2="15.24" y2="2.54" width="0.254" layer="94"/>
<wire x1="15.24" y1="-5.08" x2="5.08" y2="-5.08" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-5.08" width="0.254" layer="94"/>
<text x="6.35" y="3.81" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="6.35" y="6.35" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="R" x="0" y="0" length="middle"/>
<pin name="G" x="0" y="-2.54" length="middle"/>
<pin name="A" x="20.32" y="0" length="middle" rot="R180"/>
<pin name="B" x="20.32" y="-2.54" length="middle" rot="R180"/>
</symbol>
<symbol name="VCC">
<circle x="0" y="1.27" radius="1.27" width="0.254" layer="94"/>
<text x="-1.905" y="3.175" size="1.778" layer="96">&gt;VALUE</text>
<pin name="VCC" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
<symbol name="RESISTOR">
<wire x1="2.54" y1="1.27" x2="10.16" y2="1.27" width="0.254" layer="94"/>
<wire x1="10.16" y1="-1.27" x2="10.16" y2="1.27" width="0.254" layer="94"/>
<wire x1="10.16" y1="-1.27" x2="2.54" y2="-1.27" width="0.254" layer="94"/>
<wire x1="2.54" y1="1.27" x2="2.54" y2="-1.27" width="0.254" layer="94"/>
<text x="3.81" y="2.54" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="3.81" y="5.08" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" visible="off" length="short" direction="pas"/>
<pin name="2" x="12.7" y="0" visible="off" length="short" direction="pas" rot="R180"/>
</symbol>
<symbol name="CAPACITOR">
<wire x1="5.588" y1="2.54" x2="5.588" y2="-2.54" width="0.254" layer="94"/>
<wire x1="7.112" y1="2.54" x2="7.112" y2="-2.54" width="0.254" layer="94"/>
<wire x1="5.08" y1="0" x2="5.588" y2="0" width="0.254" layer="94"/>
<wire x1="7.112" y1="0" x2="7.62" y2="0" width="0.254" layer="94"/>
<text x="3.81" y="3.81" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="3.81" y="6.35" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" visible="off" length="middle" direction="pas"/>
<pin name="2" x="12.7" y="0" visible="off" length="middle" direction="pas" rot="R180"/>
</symbol>
<symbol name="ESP32-WROOM-32D">
<wire x1="5.08" y1="7.62" x2="48.26" y2="7.62" width="0.254" layer="94"/>
<wire x1="48.26" y1="-43.18" x2="48.26" y2="7.62" width="0.254" layer="94"/>
<wire x1="48.26" y1="-43.18" x2="5.08" y2="-43.18" width="0.254" layer="94"/>
<wire x1="5.08" y1="7.62" x2="5.08" y2="-43.18" width="0.254" layer="94"/>
<text x="6.35" y="8.89" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="6.35" y="11.43" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="GND_1" x="0" y="0" length="middle" direction="pwr"/>
<pin name="3V3" x="0" y="-2.54" length="middle" direction="pwr"/>
<pin name="EN" x="0" y="-5.08" length="middle"/>
<pin name="SENSOR_VP" x="0" y="-7.62" length="middle"/>
<pin name="SENSOR_VN" x="0" y="-10.16" length="middle"/>
<pin name="IO34" x="0" y="-12.7" length="middle"/>
<pin name="IO35" x="0" y="-15.24" length="middle"/>
<pin name="IO32" x="0" y="-17.78" length="middle"/>
<pin name="IO33" x="0" y="-20.32" length="middle"/>
<pin name="IO25" x="0" y="-22.86" length="middle"/>
<pin name="IO26" x="0" y="-25.4" length="middle"/>
<pin name="IO27" x="0" y="-27.94" length="middle"/>
<pin name="IO14" x="0" y="-30.48" length="middle"/>
<pin name="IO12" x="0" y="-33.02" length="middle"/>
<pin name="GND_2" x="15.24" y="-48.26" length="middle" direction="pwr" rot="R90"/>
<pin name="IO13" x="17.78" y="-48.26" length="middle" rot="R90"/>
<pin name="SD2" x="20.32" y="-48.26" length="middle" rot="R90"/>
<pin name="SD3" x="22.86" y="-48.26" length="middle" rot="R90"/>
<pin name="CMD" x="25.4" y="-48.26" length="middle" rot="R90"/>
<pin name="CLK" x="27.94" y="-48.26" length="middle" rot="R90"/>
<pin name="SD0" x="30.48" y="-48.26" length="middle" rot="R90"/>
<pin name="SD1" x="33.02" y="-48.26" length="middle" rot="R90"/>
<pin name="IO15" x="35.56" y="-48.26" length="middle" rot="R90"/>
<pin name="IO2" x="38.1" y="-48.26" length="middle" rot="R90"/>
<pin name="IO0" x="53.34" y="-33.02" length="middle" rot="R180"/>
<pin name="IO4" x="53.34" y="-30.48" length="middle" rot="R180"/>
<pin name="IO16" x="53.34" y="-27.94" length="middle" rot="R180"/>
<pin name="IO17" x="53.34" y="-25.4" length="middle" rot="R180"/>
<pin name="IO5" x="53.34" y="-22.86" length="middle" rot="R180"/>
<pin name="IO18" x="53.34" y="-20.32" length="middle" rot="R180"/>
<pin name="IO19" x="53.34" y="-17.78" length="middle" rot="R180"/>
<pin name="NC" x="53.34" y="-15.24" length="middle" direction="nc" rot="R180"/>
<pin name="IO21" x="53.34" y="-12.7" length="middle" rot="R180"/>
<pin name="RXD0" x="53.34" y="-10.16" length="middle" rot="R180"/>
<pin name="TXD0" x="53.34" y="-7.62" length="middle" rot="R180"/>
<pin name="IO22" x="53.34" y="-5.08" length="middle" rot="R180"/>
<pin name="IO23" x="53.34" y="-2.54" length="middle" rot="R180"/>
<pin name="GND_3" x="53.34" y="0" length="middle" direction="pwr" rot="R180"/>
<pin name="GND_4" x="53.34" y="5.08" length="middle" direction="pwr" rot="R180"/>
</symbol>
<symbol name="CONNECTOR-5P">
<wire x1="5.08" y1="2.54" x2="12.7" y2="2.54" width="0.254" layer="94"/>
<wire x1="12.7" y1="-12.7" x2="12.7" y2="2.54" width="0.254" layer="94"/>
<wire x1="12.7" y1="-12.7" x2="5.08" y2="-12.7" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-12.7" width="0.254" layer="94"/>
<text x="5.08" y="3.81" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="5.08" y="6.35" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" length="middle"/>
<pin name="2" x="0" y="-2.54" length="middle"/>
<pin name="3" x="0" y="-5.08" length="middle"/>
<pin name="4" x="0" y="-7.62" length="middle"/>
<pin name="5" x="0" y="-10.16" length="middle"/>
</symbol>
<symbol name="GND">
<wire x1="-1.905" y1="0" x2="1.905" y2="0" width="0.254" layer="94"/>
<text x="-2.54" y="-2.54" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
</symbol>
<symbol name="REED">
<wire x1="2.54" y1="1.27" x2="10.16" y2="1.27" width="0.254" layer="94"/>
<wire x1="10.16" y1="-1.27" x2="10.16" y2="1.27" width="0.254" layer="94"/>
<wire x1="10.16" y1="-1.27" x2="2.54" y2="-1.27" width="0.254" layer="94"/>
<wire x1="2.54" y1="1.27" x2="2.54" y2="-1.27" width="0.254" layer="94"/>
<text x="3.81" y="2.54" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="3.81" y="5.08" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" visible="off" length="short" direction="pas"/>
<pin name="2" x="12.7" y="0" visible="off" length="short" direction="pas" rot="R180"/>
<wire x1="2.6" y1="0" x2="5.6" y2="0" width="0.1524" layer="94"/>
<wire x1="5.6" y1="0" x2="6.4" y2="0.8" width="0.1524" layer="94"/>
<wire x1="6.4" y1="0" x2="10" y2="0" width="0.1524" layer="94"/>
</symbol>
<symbol name="BUTTON-4P">
<wire x1="2.54" y1="2.54" x2="10.16" y2="2.54" width="0.254" layer="94"/>
<wire x1="10.16" y1="-5.08" x2="10.16" y2="-2.54" width="0.254" layer="94"/>
<wire x1="10.16" y1="-2.54" x2="10.16" y2="0" width="0.254" layer="94"/>
<wire x1="10.16" y1="0" x2="10.16" y2="2.54" width="0.254" layer="94"/>
<wire x1="10.16" y1="-5.08" x2="2.54" y2="-5.08" width="0.254" layer="94"/>
<wire x1="2.54" y1="2.54" x2="2.54" y2="0" width="0.254" layer="94"/>
<text x="3.81" y="3.81" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="3.81" y="6.35" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" visible="pad" length="short" direction="pas"/>
<pin name="2" x="0" y="-2.54" visible="pad" length="short" direction="pas"/>
<pin name="4" x="12.7" y="0" visible="pad" length="short" direction="pas" rot="R180"/>
<pin name="3" x="12.7" y="-2.54" visible="pad" length="short" direction="pas" rot="R180"/>
<wire x1="2.54" y1="0" x2="2.54" y2="-2.54" width="0.254" layer="94"/>
<wire x1="2.54" y1="-2.54" x2="2.54" y2="-5.08" width="0.254" layer="94"/>
<wire x1="2.54" y1="0" x2="6.35" y2="0" width="0.1524" layer="94"/>
<wire x1="6.35" y1="0" x2="10.16" y2="0" width="0.1524" layer="94"/>
<wire x1="2.54" y1="-2.54" x2="6.35" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="6.35" y1="-2.54" x2="10.16" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="6.35" y1="0" x2="6.35" y2="-1.016" width="0.1524" layer="94"/>
<wire x1="6.35" y1="-1.016" x2="5.588" y2="-1.778" width="0.1524" layer="94"/>
<wire x1="6.35" y1="-2.54" x2="6.35" y2="-1.778" width="0.1524" layer="94"/>
</symbol>
<symbol name="BATTERY">
<wire x1="5.588" y1="2.54" x2="5.588" y2="-2.54" width="0.254" layer="94"/>
<wire x1="5.08" y1="0" x2="5.588" y2="0" width="0.254" layer="94"/>
<wire x1="7.112" y1="0" x2="7.62" y2="0" width="0.254" layer="94"/>
<text x="3.81" y="3.81" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="3.81" y="6.35" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="+" x="0" y="0" visible="off" length="middle" direction="pwr"/>
<pin name="-" x="12.7" y="0" visible="off" length="middle" direction="pwr" rot="R180"/>
<wire x1="7.112" y1="1.524" x2="7.112" y2="-1.524" width="0.254" layer="94"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="LED-RGB-3528-PLCC4" prefix="LED" uservalue="yes">
<gates>
<gate name="G$1" symbol="LED-RGB-A" x="0" y="0"/>
</gates>
<devices>
<device name="" package="LED-RGB-3528-PLCC4">
<connects>
<connect gate="G$1" pin="A" pad="4"/>
<connect gate="G$1" pin="B" pad="3"/>
<connect gate="G$1" pin="G" pad="2"/>
<connect gate="G$1" pin="R" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="VCC" prefix="SUP" uservalue="yes">
<gates>
<gate name="G$1" symbol="VCC" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="RESISTOR-0603" prefix="R" uservalue="yes">
<gates>
<gate name="G$1" symbol="RESISTOR" x="0" y="0"/>
</gates>
<devices>
<device name="" package="0603">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="CAPACITOR-0603" prefix="C" uservalue="yes">
<gates>
<gate name="G$1" symbol="CAPACITOR" x="0" y="0"/>
</gates>
<devices>
<device name="" package="0603">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="ESP32-WROOM-32D" prefix="MOD" uservalue="yes">
<gates>
<gate name="G$1" symbol="ESP32-WROOM-32D" x="0" y="0"/>
</gates>
<devices>
<device name="" package="ESP32-WROOM-32D">
<connects>
<connect gate="G$1" pin="3V3" pad="2"/>
<connect gate="G$1" pin="CLK" pad="20"/>
<connect gate="G$1" pin="CMD" pad="19"/>
<connect gate="G$1" pin="EN" pad="3"/>
<connect gate="G$1" pin="GND_1" pad="1"/>
<connect gate="G$1" pin="GND_2" pad="15"/>
<connect gate="G$1" pin="GND_3" pad="38"/>
<connect gate="G$1" pin="GND_4" pad="39"/>
<connect gate="G$1" pin="IO0" pad="25"/>
<connect gate="G$1" pin="IO12" pad="14"/>
<connect gate="G$1" pin="IO13" pad="16"/>
<connect gate="G$1" pin="IO14" pad="13"/>
<connect gate="G$1" pin="IO15" pad="23"/>
<connect gate="G$1" pin="IO16" pad="27"/>
<connect gate="G$1" pin="IO17" pad="28"/>
<connect gate="G$1" pin="IO18" pad="30"/>
<connect gate="G$1" pin="IO19" pad="31"/>
<connect gate="G$1" pin="IO2" pad="24"/>
<connect gate="G$1" pin="IO21" pad="33"/>
<connect gate="G$1" pin="IO22" pad="36"/>
<connect gate="G$1" pin="IO23" pad="37"/>
<connect gate="G$1" pin="IO25" pad="10"/>
<connect gate="G$1" pin="IO26" pad="11"/>
<connect gate="G$1" pin="IO27" pad="12"/>
<connect gate="G$1" pin="IO32" pad="8"/>
<connect gate="G$1" pin="IO33" pad="9"/>
<connect gate="G$1" pin="IO34" pad="6"/>
<connect gate="G$1" pin="IO35" pad="7"/>
<connect gate="G$1" pin="IO4" pad="26"/>
<connect gate="G$1" pin="IO5" pad="29"/>
<connect gate="G$1" pin="NC" pad="32"/>
<connect gate="G$1" pin="RXD0" pad="34"/>
<connect gate="G$1" pin="SD0" pad="21"/>
<connect gate="G$1" pin="SD1" pad="22"/>
<connect gate="G$1" pin="SD2" pad="17"/>
<connect gate="G$1" pin="SD3" pad="18"/>
<connect gate="G$1" pin="SENSOR_VN" pad="5"/>
<connect gate="G$1" pin="SENSOR_VP" pad="4"/>
<connect gate="G$1" pin="TXD0" pad="35"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="HEADER-5P-DıP-2_54MM" prefix="J" uservalue="yes">
<gates>
<gate name="G$1" symbol="CONNECTOR-5P" x="0" y="0"/>
</gates>
<devices>
<device name="" package="HEADER-5P-DıP-2_54MM">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="GND" prefix="GND" uservalue="yes">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="GND" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="REED" prefix="RD" uservalue="yes">
<gates>
<gate name="G$1" symbol="REED" x="0" y="0"/>
</gates>
<devices>
<device name="" package="0603">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="BUTTON-4P-SMD-5X5" prefix="SW" uservalue="yes">
<description>&lt;b&gt;SparkFun Accessories Mini Pushbutton Switch - SMD&lt;/b&gt;&lt;p&gt;
Source: &lt;a href="https://www.sparkfun.com/datasheets/Components/Buttons/SMD-Button.pdf"&gt; Datasheet &lt;/a&gt;</description>
<gates>
<gate name="G$1" symbol="BUTTON-4P" x="0" y="0"/>
</gates>
<devices>
<device name="" package="BUTTON-4P-SMD-5X5">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="BH-411-1P24" prefix="BAT" uservalue="yes">
<gates>
<gate name="G$1" symbol="BATTERY" x="0" y="0"/>
</gates>
<devices>
<device name="" package="BH-411-1P24">
<connects>
<connect gate="G$1" pin="+" pad="1"/>
<connect gate="G$1" pin="-" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="LED1" library="SamacSys_Parts" deviceset="LED-RGB-3528-PLCC4" device=""/>
<part name="SUP1" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="R1" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="C1" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="MOD1" library="SamacSys_Parts" deviceset="ESP32-WROOM-32D" device=""/>
<part name="J1" library="SamacSys_Parts" deviceset="HEADER-5P-DıP-2_54MM" device=""/>
<part name="GND1" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND2" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="C2" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND3" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP2" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="R2" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="C3" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="SUP3" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND4" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP4" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND5" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="R3" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R4" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="GND6" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="RD1" library="SamacSys_Parts" deviceset="REED" device=""/>
<part name="SW1" library="SamacSys_Parts" deviceset="BUTTON-4P-SMD-5X5" device=""/>
<part name="R5" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="C4" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="SUP5" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND7" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="R6" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="C5" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND8" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP6" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND9" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND10" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="BAT1" library="SamacSys_Parts" deviceset="BH-411-1P24" device=""/>
<part name="SUP7" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND11" library="SamacSys_Parts" deviceset="GND" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="LED1" gate="G$1" x="-25.4" y="-45.72" smashed="yes">
<attribute name="NAME" x="-19.05" y="-41.91" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-19.05" y="-39.37" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="SUP1" gate="G$1" x="-2.54" y="-40.64" smashed="yes">
<attribute name="VALUE" x="-4.445" y="-37.465" size="1.778" layer="96"/>
</instance>
<instance part="R1" gate="G$1" x="-45.72" y="17.78" smashed="yes" rot="R90">
<attribute name="NAME" x="-48.26" y="21.59" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-50.8" y="21.59" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="C1" gate="G$1" x="-40.64" y="38.1" smashed="yes">
<attribute name="NAME" x="-36.83" y="41.91" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-36.83" y="44.45" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="MOD1" gate="G$1" x="-25.4" y="17.78" smashed="yes">
<attribute name="NAME" x="-19.05" y="26.67" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-19.05" y="29.21" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="J1" gate="G$1" x="50.8" y="15.24" smashed="yes" rot="R90">
<attribute name="NAME" x="46.99" y="20.32" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="44.45" y="20.32" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND1" gate="1" x="38.1" y="25.4" smashed="yes" rot="R180">
<attribute name="VALUE" x="40.64" y="27.94" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND2" gate="1" x="-35.56" y="25.4" smashed="yes" rot="R180">
<attribute name="VALUE" x="-33.02" y="27.94" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="C2" gate="G$1" x="-40.64" y="30.48" smashed="yes">
<attribute name="NAME" x="-36.83" y="34.29" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-36.83" y="36.83" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="GND3" gate="1" x="-22.86" y="27.94" smashed="yes">
<attribute name="VALUE" x="-25.4" y="25.4" size="1.778" layer="96"/>
</instance>
<instance part="SUP2" gate="G$1" x="-40.64" y="43.18" smashed="yes">
<attribute name="VALUE" x="-42.545" y="46.355" size="1.778" layer="96"/>
</instance>
<instance part="R2" gate="G$1" x="-27.94" y="-43.18" smashed="yes" rot="R90">
<attribute name="NAME" x="-30.48" y="-39.37" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-33.02" y="-39.37" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="C3" gate="G$1" x="-58.42" y="12.7" smashed="yes">
<attribute name="NAME" x="-54.61" y="16.51" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-54.61" y="19.05" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="SUP3" gate="G$1" x="-45.72" y="43.18" smashed="yes">
<attribute name="VALUE" x="-47.625" y="46.355" size="1.778" layer="96"/>
</instance>
<instance part="GND4" gate="1" x="-66.04" y="12.7" smashed="yes" rot="R270">
<attribute name="VALUE" x="-68.58" y="15.24" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="SUP4" gate="G$1" x="58.42" y="10.16" smashed="yes" rot="R180">
<attribute name="VALUE" x="60.325" y="6.985" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND5" gate="1" x="66.04" y="15.24" smashed="yes" rot="R90">
<attribute name="VALUE" x="68.58" y="12.7" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="R3" gate="G$1" x="-33.02" y="-43.18" smashed="yes" rot="R90">
<attribute name="NAME" x="-35.56" y="-39.37" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-38.1" y="-39.37" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="R4" gate="G$1" x="-38.1" y="-43.18" smashed="yes" rot="R90">
<attribute name="NAME" x="-40.64" y="-39.37" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-43.18" y="-39.37" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND6" gate="1" x="-10.16" y="-33.02" smashed="yes">
<attribute name="VALUE" x="-12.7" y="-35.56" size="1.778" layer="96"/>
</instance>
<instance part="RD1" gate="G$1" x="-58.42" y="-7.62" smashed="yes">
<attribute name="NAME" x="-54.61" y="-5.08" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-54.61" y="-2.54" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="SW1" gate="G$1" x="-58.42" y="-30.48" smashed="yes">
<attribute name="NAME" x="-54.61" y="-26.67" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-54.61" y="-24.13" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R5" gate="G$1" x="-58.42" y="-2.54" smashed="yes">
<attribute name="NAME" x="-54.61" y="0" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-54.61" y="2.54" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="C4" gate="G$1" x="-58.42" y="5.08" smashed="yes">
<attribute name="NAME" x="-54.61" y="8.89" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-54.61" y="11.43" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="SUP5" gate="G$1" x="-63.5" y="-7.62" smashed="yes" rot="R90">
<attribute name="VALUE" x="-66.675" y="-9.525" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND7" gate="1" x="-66.04" y="-2.54" smashed="yes" rot="R270">
<attribute name="VALUE" x="-68.58" y="0" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="R6" gate="G$1" x="-58.42" y="-22.86" smashed="yes">
<attribute name="NAME" x="-54.61" y="-20.32" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-54.61" y="-17.78" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="C5" gate="G$1" x="-58.42" y="-15.24" smashed="yes">
<attribute name="NAME" x="-54.61" y="-11.43" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-54.61" y="-8.89" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="GND8" gate="1" x="-66.04" y="-22.86" smashed="yes" rot="R270">
<attribute name="VALUE" x="-68.58" y="-20.32" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="SUP6" gate="G$1" x="-45.72" y="-38.1" smashed="yes" rot="MR180">
<attribute name="VALUE" x="-47.625" y="-41.275" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="GND9" gate="1" x="-66.04" y="5.08" smashed="yes" rot="R270">
<attribute name="VALUE" x="-68.58" y="7.62" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="GND10" gate="1" x="-66.04" y="-15.24" smashed="yes" rot="R270">
<attribute name="VALUE" x="-68.58" y="-12.7" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="BAT1" gate="G$1" x="5.08" y="-38.1" smashed="yes" rot="R270">
<attribute name="NAME" x="8.89" y="-41.91" size="1.778" layer="95" rot="R270" align="center-left"/>
<attribute name="VALUE" x="11.43" y="-41.91" size="1.778" layer="96" rot="R270" align="center-left"/>
</instance>
<instance part="SUP7" gate="G$1" x="5.08" y="-35.56" smashed="yes">
<attribute name="VALUE" x="3.175" y="-32.385" size="1.778" layer="96"/>
</instance>
<instance part="GND11" gate="1" x="5.08" y="-53.34" smashed="yes">
<attribute name="VALUE" x="2.54" y="-55.88" size="1.778" layer="96"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="GND" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="GND_4"/>
<pinref part="GND1" gate="1" pin="GND"/>
<wire x1="27.94" y1="22.86" x2="33.02" y2="22.86" width="0.1524" layer="91"/>
<pinref part="MOD1" gate="G$1" pin="GND_3"/>
<wire x1="33.02" y1="22.86" x2="38.1" y2="22.86" width="0.1524" layer="91"/>
<wire x1="27.94" y1="17.78" x2="33.02" y2="17.78" width="0.1524" layer="91"/>
<wire x1="33.02" y1="17.78" x2="33.02" y2="22.86" width="0.1524" layer="91"/>
<junction x="33.02" y="22.86"/>
</segment>
<segment>
<pinref part="MOD1" gate="G$1" pin="GND_1"/>
<pinref part="GND2" gate="1" pin="GND"/>
<wire x1="-25.4" y1="17.78" x2="-35.56" y2="17.78" width="0.1524" layer="91"/>
<wire x1="-35.56" y1="17.78" x2="-35.56" y2="22.86" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C1" gate="G$1" pin="2"/>
<pinref part="C2" gate="G$1" pin="2"/>
<wire x1="-27.94" y1="38.1" x2="-27.94" y2="30.48" width="0.1524" layer="91"/>
<pinref part="GND3" gate="1" pin="GND"/>
<wire x1="-27.94" y1="30.48" x2="-22.86" y2="30.48" width="0.1524" layer="91"/>
<junction x="-27.94" y="30.48"/>
</segment>
<segment>
<pinref part="GND4" gate="1" pin="GND"/>
<pinref part="C3" gate="G$1" pin="1"/>
<wire x1="-63.5" y1="12.7" x2="-58.42" y2="12.7" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="J1" gate="G$1" pin="5"/>
<pinref part="GND5" gate="1" pin="GND"/>
<wire x1="60.96" y1="15.24" x2="63.5" y2="15.24" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="MOD1" gate="G$1" pin="GND_2"/>
<pinref part="GND6" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="R5" gate="G$1" pin="1"/>
<pinref part="GND7" gate="1" pin="GND"/>
<wire x1="-58.42" y1="-2.54" x2="-63.5" y2="-2.54" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND10" gate="1" pin="GND"/>
<pinref part="C5" gate="G$1" pin="1"/>
<wire x1="-58.42" y1="-15.24" x2="-63.5" y2="-15.24" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="R6" gate="G$1" pin="1"/>
<pinref part="GND8" gate="1" pin="GND"/>
<wire x1="-63.5" y1="-22.86" x2="-58.42" y2="-22.86" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C4" gate="G$1" pin="1"/>
<pinref part="GND9" gate="1" pin="GND"/>
<wire x1="-58.42" y1="5.08" x2="-63.5" y2="5.08" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="BAT1" gate="G$1" pin="-"/>
<pinref part="GND11" gate="1" pin="GND"/>
</segment>
</net>
<net name="VCC" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="3V3"/>
<pinref part="C2" gate="G$1" pin="1"/>
<wire x1="-25.4" y1="15.24" x2="-40.64" y2="15.24" width="0.1524" layer="91"/>
<wire x1="-40.64" y1="15.24" x2="-40.64" y2="30.48" width="0.1524" layer="91"/>
<pinref part="C1" gate="G$1" pin="1"/>
<wire x1="-40.64" y1="30.48" x2="-40.64" y2="38.1" width="0.1524" layer="91"/>
<junction x="-40.64" y="30.48"/>
<pinref part="SUP2" gate="G$1" pin="VCC"/>
<wire x1="-40.64" y1="40.64" x2="-40.64" y2="38.1" width="0.1524" layer="91"/>
<junction x="-40.64" y="38.1"/>
</segment>
<segment>
<pinref part="SUP3" gate="G$1" pin="VCC"/>
<pinref part="R1" gate="G$1" pin="2"/>
<wire x1="-45.72" y1="40.64" x2="-45.72" y2="30.48" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="SUP4" gate="G$1" pin="VCC"/>
<pinref part="J1" gate="G$1" pin="4"/>
<wire x1="58.42" y1="12.7" x2="58.42" y2="15.24" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="LED1" gate="G$1" pin="A"/>
<pinref part="SUP1" gate="G$1" pin="VCC"/>
<wire x1="-5.08" y1="-45.72" x2="-2.54" y2="-45.72" width="0.1524" layer="91"/>
<wire x1="-2.54" y1="-45.72" x2="-2.54" y2="-43.18" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="RD1" gate="G$1" pin="1"/>
<pinref part="SUP5" gate="G$1" pin="VCC"/>
<wire x1="-58.42" y1="-7.62" x2="-60.96" y2="-7.62" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="SW1" gate="G$1" pin="3"/>
<pinref part="SUP6" gate="G$1" pin="VCC"/>
<wire x1="-45.72" y1="-33.02" x2="-45.72" y2="-35.56" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="BAT1" gate="G$1" pin="+"/>
<pinref part="SUP7" gate="G$1" pin="VCC"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="R1" gate="G$1" pin="1"/>
<wire x1="-45.72" y1="17.78" x2="-45.72" y2="12.7" width="0.1524" layer="91"/>
<pinref part="MOD1" gate="G$1" pin="EN"/>
<wire x1="-45.72" y1="12.7" x2="-25.4" y2="12.7" width="0.1524" layer="91"/>
<pinref part="C3" gate="G$1" pin="2"/>
<junction x="-45.72" y="12.7"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="TXD0"/>
<pinref part="J1" gate="G$1" pin="1"/>
<wire x1="27.94" y1="10.16" x2="50.8" y2="10.16" width="0.1524" layer="91"/>
<wire x1="50.8" y1="10.16" x2="50.8" y2="15.24" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="RXD0"/>
<pinref part="J1" gate="G$1" pin="2"/>
<wire x1="27.94" y1="7.62" x2="53.34" y2="7.62" width="0.1524" layer="91"/>
<wire x1="53.34" y1="7.62" x2="53.34" y2="15.24" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="IO0"/>
<pinref part="J1" gate="G$1" pin="3"/>
<wire x1="27.94" y1="-15.24" x2="55.88" y2="-15.24" width="0.1524" layer="91"/>
<wire x1="55.88" y1="-15.24" x2="55.88" y2="15.24" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$8" class="0">
<segment>
<pinref part="R5" gate="G$1" pin="2"/>
<pinref part="RD1" gate="G$1" pin="2"/>
<wire x1="-45.72" y1="-2.54" x2="-45.72" y2="-7.62" width="0.1524" layer="91"/>
<junction x="-45.72" y="-2.54"/>
<pinref part="C4" gate="G$1" pin="2"/>
<wire x1="-45.72" y1="5.08" x2="-45.72" y2="-2.54" width="0.1524" layer="91"/>
<wire x1="-45.72" y1="-2.54" x2="-43.18" y2="-2.54" width="0.1524" layer="91"/>
<wire x1="-43.18" y1="-2.54" x2="-43.18" y2="0" width="0.1524" layer="91"/>
<pinref part="MOD1" gate="G$1" pin="IO32"/>
<wire x1="-43.18" y1="0" x2="-25.4" y2="0" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<pinref part="R2" gate="G$1" pin="1"/>
<pinref part="LED1" gate="G$1" pin="R"/>
<wire x1="-27.94" y1="-43.18" x2="-27.94" y2="-45.72" width="0.1524" layer="91"/>
<wire x1="-27.94" y1="-45.72" x2="-25.4" y2="-45.72" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<pinref part="R3" gate="G$1" pin="1"/>
<pinref part="LED1" gate="G$1" pin="G"/>
<wire x1="-33.02" y1="-43.18" x2="-33.02" y2="-48.26" width="0.1524" layer="91"/>
<wire x1="-33.02" y1="-48.26" x2="-25.4" y2="-48.26" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<pinref part="R4" gate="G$1" pin="1"/>
<wire x1="-38.1" y1="-43.18" x2="-38.1" y2="-53.34" width="0.1524" layer="91"/>
<wire x1="-38.1" y1="-53.34" x2="-2.54" y2="-53.34" width="0.1524" layer="91"/>
<wire x1="-2.54" y1="-53.34" x2="-2.54" y2="-48.26" width="0.1524" layer="91"/>
<pinref part="LED1" gate="G$1" pin="B"/>
<wire x1="-2.54" y1="-48.26" x2="-5.08" y2="-48.26" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$10" class="0">
<segment>
<pinref part="R2" gate="G$1" pin="2"/>
<pinref part="MOD1" gate="G$1" pin="IO27"/>
<wire x1="-27.94" y1="-30.48" x2="-27.94" y2="-10.16" width="0.1524" layer="91"/>
<wire x1="-27.94" y1="-10.16" x2="-25.4" y2="-10.16" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$11" class="0">
<segment>
<pinref part="R3" gate="G$1" pin="2"/>
<pinref part="MOD1" gate="G$1" pin="IO26"/>
<wire x1="-33.02" y1="-30.48" x2="-33.02" y2="-7.62" width="0.1524" layer="91"/>
<wire x1="-33.02" y1="-7.62" x2="-25.4" y2="-7.62" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$12" class="0">
<segment>
<pinref part="R4" gate="G$1" pin="2"/>
<pinref part="MOD1" gate="G$1" pin="IO25"/>
<wire x1="-38.1" y1="-30.48" x2="-38.1" y2="-5.08" width="0.1524" layer="91"/>
<wire x1="-38.1" y1="-5.08" x2="-25.4" y2="-5.08" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$13" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="IO33"/>
<wire x1="-25.4" y1="-2.54" x2="-40.64" y2="-2.54" width="0.1524" layer="91"/>
<wire x1="-40.64" y1="-2.54" x2="-40.64" y2="-22.86" width="0.1524" layer="91"/>
<pinref part="SW1" gate="G$1" pin="4"/>
<pinref part="R6" gate="G$1" pin="2"/>
<wire x1="-45.72" y1="-22.86" x2="-45.72" y2="-30.48" width="0.1524" layer="91"/>
<junction x="-45.72" y="-22.86"/>
<pinref part="C5" gate="G$1" pin="2"/>
<wire x1="-45.72" y1="-15.24" x2="-45.72" y2="-22.86" width="0.1524" layer="91"/>
<wire x1="-40.64" y1="-22.86" x2="-45.72" y2="-22.86" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
