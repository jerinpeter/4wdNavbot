/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, EAIBOT, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/** @mainpage CYdLidar(YDLIDAR SDK API)
    <table>
        <tr><th>Library     <td>CYdLidar
        <tr><th>File        <td>CYdLidar.h
        <tr><th>Author      <td>Tony [code at ydlidar com]
        <tr><th>Source      <td>https://github.com/ydlidar/YDLidar-SDK
        <tr><th>Version     <td>1.0.0
        <tr><th>Sample      <td>[ydlidar test](\ref samples/main.cpp)[G1 G2 G4 G6 S2 X2 X4)\n
    </table>
    This API calls Two LiDAR interface classes in the following sections:
        - @subpage YDlidarDriver

* @copyright    Copyright (c) 2018-2020  EAIBOT

    Jump to the @link ::CYdLidar @endlink interface documentation.

*/

#pragma once
#include "utils.h"
#include "ydlidar_driver.h"
#include <math.h>

using namespace ydlidar;

/**
 * @ref "Dataset"
 * @par Dataset:
<table>
<tr><th>LIDAR      <th> Model  <th>  Baudrate <th>  SampleRate(K) <th> Range(m)  		<th>  Frequency(HZ) <th> Intenstiy(bit) <th> SingleChannel<th> voltage(V)
<tr><th> F4        <td> 1	   <td>  115200   <td>   4            <td>  0.12~12         <td> 5~12           <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> S4        <td> 4	   <td>  115200   <td>   4            <td>  0.10~8.0        <td> 5~12 (PWM)     <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> S4B       <td> 4/11   <td>  153600   <td>   4            <td>  0.10~8.0        <td> 5~12(PWM)      <td> true(8)        <td> false    	  <td> 4.8~5.2
<tr><th> S2        <td> 4/12   <td>  115200   <td>   3            <td>  0.10~8.0     	<td> 4~8(PWM)       <td> false          <td> true    	  <td> 4.8~5.2
<tr><th> G4        <td> 5	   <td>  230400   <td>   9/8/4        <td>  0.28/0.26/0.1~16<td> 5~12        	<td> false          <td> false    	  <td> 4.8~5.2
<tr><th> X4        <td> 6	   <td>  128000   <td>   5            <td>  0.12~10     	<td> 5~12(PWM)      <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> X2/X2L    <td> 6	   <td>  115200   <td>   3            <td>  0.10~8.0     	<td> 4~8(PWM)       <td> false          <td> true    	  <td> 4.8~5.2
<tr><th> G4PRO     <td> 7	   <td>  230400   <td>   9/8/4        <td>  0.28/0.26/0.1~16<td> 5~12        	<td> false          <td> false    	  <td> 4.8~5.2
<tr><th> F4PRO     <td> 8	   <td>  230400   <td>   4/6          <td>  0.12~12         <td> 5~12        	<td> false          <td> false    	  <td> 4.8~5.2
<tr><th> R2        <td> 9	   <td>  230400   <td>   5            <td>  0.12~16         <td> 5~12        	<td> false          <td> false    	  <td> 4.8~5.2
<tr><th> G6        <td> 13     <td>  512000   <td>   18/16/8      <td>  0.28/0.26/0.1~25<td> 5~12        	<td> false          <td> false    	  <td> 4.8~5.2
<tr><th> G2A       <td> 14	   <td>  230400   <td>   5            <td>  0.12~12         <td> 5~12      	    <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> G2        <td> 15     <td>  230400   <td>   5            <td>  0.28~16     	<td> 5~12      	    <td> true(8)        <td> false    	  <td> 4.8~5.2
<tr><th> G2C       <td> 16	   <td>  115200   <td>   4            <td>  0.1~12        	<td> 5~12      	    <td> false      	<td> false    	  <td> 4.8~5.2
<tr><th> G4B       <td> 17	   <td>  512000   <td>   10           <td>  0.12~16         <td> 5~12        	<td> true(10)       <td> false    	  <td> 4.8~5.2
<tr><th> G4C       <td> 18	   <td>  115200   <td>   4            <td>  0.1~12		    <td> 5~12           <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> G1        <td> 19	   <td>  230400   <td>   9            <td>  0.28~16         <td> 5~12      	    <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> TX8    　 <td> 100	   <td>  115200   <td>   4            <td>  0.01~8      	<td> 4~8(PWM)       <td> false          <td> true      	  <td> 4.8~5.2
<tr><th> TX20    　<td> 100	   <td>  115200   <td>   4            <td>  0.01~8      	<td> 4~8(PWM)       <td> false          <td> true     	  <td> 4.8~5.2
<tr><th> TG15    　<td> 100	   <td>  512000   <td>   20/18/10     <td>  0.01~30      	<td> 3~16      	    <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> TG30    　<td> 101	   <td>  512000   <td>   20/18/10     <td>  0.01~30      	<td> 3~16      	    <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> TG50    　<td> 102	   <td>  512000   <td>   20/18/10     <td>  0.01~50      	<td> 3~16      	    <td> false          <td> false    	  <td> 4.8~5.2
</table>
 */

/**
 * @par example: G4 LiDAR
 * @code
 *    ///< Defining an CYdLidar instance.
 *    CYdLidar laser;
 *    ///< LiDAR Maximum angle
 *    laser.setMaxAngle(180);
 *    ///< LiDAR Minimum angle
 *    laser.setMinAngle(-180);
 *    /// LiDAR Minimum range
 *    laser.setMinRange(0.1);
 *    /// LiDAR Maximum range
 *    laser.setMaxRange(16.0);
 *    ///< LiDAR serial port
 *    laser.setSerialPort("/dev/ydlidar");
 *    ///< G4 LiDAR baudrate
 *    laser.setSerialBaudrate(230400);
 *    ///< Fixed angle resolution
 *    laser.setFixedResolution(false);
 *    ///< rotate 180 degress
 *    laser.setReversion(true);
 *    ///< LiDAR Direction Counterclockwise
 *    laser.setInverted(true);
 *    ///< LiDAR Scan frequency
 *    laser.setScanFrequency(10.0);
 *    ///< LiDAR sample rate
 *    laser.setSampleRate(9);
 *    ///< LiDAR hot plug
 *    laser.setAutoReconnect(true);
 *    ///< LiDAR ignore array
 *    std::vector<float> ignore_array;
 *    laser.setIgnoreArray(ignore_array);
 *    ///LiDAR communication type
 *    laser.setSingleChannel(false);
 *    ///LiDAR Type
 *    laser.setLidarType(TYPE_TRIANGLE);
 *    /// LiDAR connection type
 *    laser.setDeviceType(YDLIDAR_TYPE_SERIAL);
 *    /// LiDAR intensity
 *    laser.setIntensity(false);
 *    /// LiDAR abnormal check count
 *    laser.setAbnormalCheckCount(4);
 *    /// LiDAR Motor DTR
 *    laser.setSupportMotorDtrCtrl(false);
 * @endcode
 */

/**
 * @par example: S2 LiDAR
 * @code
 *    ///< Defining an CYdLidar instance.
 *    CYdLidar laser;
 *    ///< LiDAR Maximum angle
 *    laser.setMaxAngle(180);
 *    ///< LiDAR Minimum angle
 *    laser.setMinAngle(-180);
 *    /// LiDAR Minimum range
 *    laser.setMinRange(0.1);
 *    /// LiDAR Maximum range
 *    laser.setMaxRange(8.0);
 *    ///< LiDAR serial port
 *    laser.setSerialPort("/dev/ydlidar");
 *    ///< G4 LiDAR baudrate
 *    laser.setSerialBaudrate(115200);
 *    ///< Fixed angle resolution
 *    laser.setFixedResolution(false);
 *    ///< rotate 180 degress
 *    laser.setReversion(false);
 *    ///< LiDAR Direction Counterclockwise
 *    laser.setInverted(true);
 *    ///< LiDAR Scan frequency, external PWM
 *    laser.setScanFrequency(6.0);
 *    ///< LiDAR sample rate
 *    laser.setSampleRate(3);
 *    ///< LiDAR hot plug
 *    laser.setAutoReconnect(true);
 *    ///< LiDAR ignore array
 *    std::vector<float> ignore_array;
 *    laser.setIgnoreArray(ignore_array);
 *    ///LiDAR communication type
 *    laser.setSingleChannel(true);
 *    ///LiDAR Type
 *    laser.setLidarType(TYPE_TRIANGLE);
 *    /// LiDAR connection type
 *    laser.setDeviceType(YDLIDAR_TYPE_SERIAL);
 *    /// LiDAR intensity
 *    laser.setIntensity(false);
 *    /// LiDAR abnormal check count
 *    laser.setAbnormalCheckCount(4);
 *    /// LiDAR Motor DTR
 *    laser.setSupportMotorDtrCtrl(true);
 * @endcode
 */


/**
 * @par example: TG30 LiDAR
 * @code
 *    ///< Defining an CYdLidar instance.
 *    CYdLidar laser;
 *    ///< LiDAR Maximum angle
 *    laser.setMaxAngle(180);
 *    ///< LiDAR Minimum angle
 *    laser.setMinAngle(-180);
 *    /// LiDAR Minimum range
 *    laser.setMinRange(0.01);
 *    /// LiDAR Maximum range
 *    laser.setMaxRange(32.0);
 *    ///< LiDAR serial port
 *    laser.setSerialPort("/dev/ydlidar");
 *    ///< G4 LiDAR baudrate
 *    laser.setSerialBaudrate(512000);
 *    ///< Fixed angle resolution
 *    laser.setFixedResolution(false);
 *    ///< rotate 180 degress
 *    laser.setReversion(true);
 *    ///< LiDAR Direction Counterclockwise
 *    laser.setInverted(true);
 *    ///< LiDAR Scan frequency
 *    laser.setScanFrequency(10.0);
 *    ///< LiDAR sample rate
 *    laser.setSampleRate(20);
 *    ///< LiDAR hot plug
 *    laser.setAutoReconnect(true);
 *    ///< LiDAR ignore array
 *    std::vector<float> ignore_array;
 *    laser.setIgnoreArray(ignore_array);
 *    ///LiDAR communication type
 *    laser.setSingleChannel(false);
 *    ///LiDAR Type
 *    laser.setLidarType(TYPE_TOF);
 *    /// LiDAR connection type
 *    laser.setDeviceType(YDLIDAR_TYPE_SERIAL);
 *    /// LiDAR intensity
 *    laser.setIntensity(false);
 *    /// LiDAR abnormal check count
 *    laser.setAbnormalCheckCount(4);
 *    /// LiDAR Motor DTR
 *    laser.setSupportMotorDtrCtrl(false);
 * @endcode
 */

/**
 * @par example: TX8 LiDAR
 * @code
 *    ///< Defining an CYdLidar instance.
 *    CYdLidar laser;
 *    ///< LiDAR Maximum angle
 *    laser.setMaxAngle(180);
 *    ///< LiDAR Minimum angle
 *    laser.setMinAngle(-180);
 *    /// LiDAR Minimum range
 *    laser.setMinRange(0.1);
 *    /// LiDAR Maximum range
 *    laser.setMaxRange(8.0);
 *    ///< LiDAR serial port
 *    laser.setSerialPort("/dev/ydlidar");
 *    ///< G4 LiDAR baudrate
 *    laser.setSerialBaudrate(115200);
 *    ///< Fixed angle resolution
 *    laser.setFixedResolution(false);
 *    ///< rotate 180 degress
 *    laser.setReversion(false);
 *    ///< LiDAR Direction Counterclockwise
 *    laser.setInverted(true);
 *    ///< LiDAR Scan frequency, external PWM
 *    laser.setScanFrequency(6.0);
 *    ///< LiDAR sample rate
 *    laser.setSampleRate(4);
 *    ///< LiDAR hot plug
 *    laser.setAutoReconnect(true);
 *    ///< LiDAR ignore array
 *    std::vector<float> ignore_array;
 *    laser.setIgnoreArray(ignore_array);
 *    ///LiDAR communication type
 *    laser.setSingleChannel(true);
 *    ///LiDAR Type
 *    laser.setLidarType(TYPE_TOF);
 *    /// LiDAR connection type
 *    laser.setDeviceType(YDLIDAR_TYPE_SERIAL);
 *    /// LiDAR intensity
 *    laser.setIntensity(false);
 *    /// LiDAR abnormal check count
 *    laser.setAbnormalCheckCount(4);
 *    /// LiDAR Motor DTR
 *    laser.setSupportMotorDtrCtrl(true);
 * @endcode
 */


/**
 * @par example: T15 LiDAR
 * @code
 *    ///< Defining an CYdLidar instance.
 *    CYdLidar laser;
 *    ///< LiDAR Maximum angle
 *    laser.setMaxAngle(180);
 *    ///< LiDAR Minimum angle
 *    laser.setMinAngle(-180);
 *    /// LiDAR Minimum range
 *    laser.setMinRange(0.01);
 *    /// LiDAR Maximum range
 *    laser.setMaxRange(64.0);
 *    ///< LiDAR serial port
 *    laser.setSerialPort("192.168.1.11");
 *    ///< G4 LiDAR baudrate
 *    laser.setSerialBaudrate(8000);
 *    ///< Fixed angle resolution
 *    laser.setFixedResolution(false);
 *    ///< rotate 180 degress
 *    laser.setReversion(true);
 *    ///< LiDAR Direction Counterclockwise
 *    laser.setInverted(true);
 *    ///< LiDAR Scan frequency
 *    laser.setScanFrequency(20.0);
 *    ///< LiDAR sample rate
 *    laser.setSampleRate(20);
 *    ///< LiDAR hot plug
 *    laser.setAutoReconnect(true);
 *    ///< LiDAR ignore array
 *    std::vector<float> ignore_array;
 *    laser.setIgnoreArray(ignore_array);
 *    ///LiDAR communication type
 *    laser.setSingleChannel(false);
 *    ///LiDAR Type
 *    laser.setLidarType(TYPE_TOF_NET);
 *    /// LiDAR connection type
 *    laser.setDeviceType(YDLIDAR_TYPE_TCP);
 *    /// LiDAR intensity
 *    laser.setIntensity(true);
 *    /// LiDAR abnormal check count
 *    laser.setAbnormalCheckCount(4);
 *    /// LiDAR Motor DTR
 *    laser.setSupportMotorDtrCtrl(false);
 * @endcode
 */



/// Provides a platform independent class to for LiDAR development.
/// This class is designed to serial or socket communication development in a
/// platform independent manner.
/// - LiDAR types
///  -# ydlidar::YDlidarDriver Class
///  -# ydlidar::ETLidarDriver Class
///

class YDLIDAR_API CYdLidar {
  /**
   * @brief Set and Get LiDAR Maximum effective range.
   * @note The effective range beyond the maxmum is set to zero.\n
   * the MaxRange should be greater than the MinRange.
   * @remarks unit: m
   * @see ::PropertyBuilderByName and [DataSet](\ref Dataset)
   * @see CYdLidar::setMaxRange and CYdLidar::getMaxRange
   */
  PropertyBuilderByName(float, MaxRange, private);
  /**
   * @brief Set and Get LiDAR Minimum effective range.
   * @note The effective range less than the minmum is set to zero.\n
   * the MinRange should be less than the MaxRange.
   * @remarks unit: m
   * @see ::PropertyBuilderByName and Dataset
   * @see CYdLidar::setMinRange and CYdLidar::getMinRange
   */
  PropertyBuilderByName(float, MinRange,private);
  /**
   * @brief Set and Get LiDAR Maximum effective angle.
   * @note The effective angle beyond the maxmum will be ignored.\n
   * the MaxAngle should be greater than the MinAngle
   * @remarks unit: degree, Range:-180~180
   * @see ::PropertyBuilderByName and Dataset
   * @see CYdLidar::setMaxAngle and CYdLidar::getMaxAngle
   */
  PropertyBuilderByName(float, MaxAngle, private);
  /**
   * @brief Set and Get LiDAR Minimum effective angle.
   * @note The effective angle less than the minmum will be ignored.\n
   * the MinAngle should be less than the MaxAngle
   * @remarks unit: degree, Range:-180~180
   * @see ::PropertyBuilderByName and Dataset
   * @see CYdLidar::setMinAngle and CYdLidar::getMinAngle
   */
  PropertyBuilderByName(float, MinAngle, private);
  /**
   * @brief Set and Get LiDAR Sampling rate.
   * @note If the set sampling rate does no exist.
   * the actual sampling rate is the LiDAR's default sampling rate.\n
   * Set the sampling rate to match the LiDAR.
   * @remarks unit: kHz/s, Ranges: 2,3,4,5,6,8,9,10,16,18,20\n
   <table>
        <tr><th>G4/F4               <td>4,8,9
        <tr><th>F4PRO               <td>4,6
        <tr><th>G6                  <td>8,16,18
        <tr><th>G4B                 <td>10
        <tr><th>G1                  <td>9
        <tr><th>G2A/G2/R2/X4        <td>5
        <tr><th>S4/S4B/G4C/TX8/TX20 <td>4
        <tr><th>G2C                 <td>4
        <tr><th>S2                  <td>3
        <tr><th>TG15/TG30/TG50      <td>10,18,20
        <tr><th>T5/T15              <td>20
    </table>
   * @see CYdLidar::setSampleRate and CYdLidar::getSampleRate
   */
  PropertyBuilderByName(int, SampleRate, private);
  /**
   * @brief Set and Get LiDAR Scan frequency.
   * @note If the LiDAR is a single channel,
   * the scanning frequency nneds to be adjusted by external PWM.\n
   * Set the scan frequency to match the LiDAR.
   * @remarks unit: Hz\n
   <table>
        <tr><th>S2/X2/X2L/TX8/TX20              <td>4~8(PWM)
        <tr><th>F4/F4PRO/G4/G4PRO/R2            <td>5~12
        <tr><th>G6/G2A/G2/G2C/G4B/G4C/G1        <td>5~12
        <tr><th>S4/S4B/X4                       <td>5~12(PWM)
        <tr><th>TG15/TG30/TG50                  <td>3~16
        <tr><th>T5/T15                          <td>5~40
    </table>
   * @see CYdLidar::setScanFrequency and CYdLidar::getScanFrequency
   */
  PropertyBuilderByName(float, ScanFrequency, private);
  /**
   * @brief Set and Get LiDAR Fixed angluar resolution.\n
   * @note The Lidar scanning frequency will change slightly due to various reasons.
   * so the number of points per circle will also change slightly.\n
   * if a fixed angluar resolution is required.
   * a fixed number of points is required.
   * @details If set to true,
   * the angle_increment of the fixed angle resolution in LaserConfig will be a fixed value.
   * @see CYdLidar::setFixedResolution and CYdLidar::getFixedResolution
   */
  PropertyBuilderByName(bool, FixedResolution, private);
  /**
   * @brief Set and Get LiDAR Reversion.\n
   * true: LiDAR data rotated 180 degrees.\n
   * false: Keep raw Data.\n
   * default: false\n
   * @note Refer to the table below for the LiDAR Reversion.\n
   * This is currently related to your coordinate system and install direction.
   * Whether to reverse it depends on your actual scene.
   * @par Reversion Table
   <table>
        <tr><th>LiDAR                           <th>reversion
        <tr><th>G1/G2/G2A/G2C/F4/F4PRO/R2       <td>true
        <tr><th>G4/G4PRO/G4B/G4C/G6             <td>true
        <tr><th>TG15/TG30/TG50                  <td>true
        <tr><th>T5/T15                          <td>true
        <tr><th>S2/X2/X2L/X4/S4/S4B             <td>false
        <tr><th>TX8/TX20                        <td>false
    </table>
   * @see CYdLidar::setReversion and CYdLidar::getReversion
   */
  PropertyBuilderByName(bool, Reversion, private);
  /**
   * @brief Set and Get LiDAR inverted.\n
   * true: Data is counterclockwise\n
   * false: Data is clockwise\n
   * Default: clockwise
   * @note If set to true, LiDAR data direction is positive counterclockwise.
   * otherwise it is positive clockwise.
   * @see CYdLidar::setInverted and CYdLidar::getInverted
   */
  PropertyBuilderByName(bool, Inverted, private);
  /**
   * @brief Set and Get LiDAR Automatically reconnect flag.\n
   * Whether to support hot plug.
   * @see CYdLidar::setAutoReconnect and CYdLidar::getAutoReconnect
   */
  PropertyBuilderByName(bool, AutoReconnect, private);
  /**
  * @brief Set and Get LiDAR baudrate or network port.
  * @note Refer to the table below for the LiDAR Baud Rate.\n
  * Set the baudrate or network port to match the LiDAR.
  * @remarks
  <table>
       <tr><th>F4/S2/X2/X2L/S4/TX8/TX20/G4C        <td>115200
       <tr><th>X4                                  <td>128000
       <tr><th>S4B                                 <td>153600
       <tr><th>G1/G2/R2/G4/G4PRO/F4PRO             <td>230400
       <tr><th>G2A/G2C                             <td>230400
       <tr><th>G6/G4B/TG15/TG30/TG50               <td>512000
       <tr><th>T5/T15(network)                     <td>8000
   </table>
  * @see CYdLidar::setSerialBaudrate and CYdLidar::getSerialBaudrate
  */
  PropertyBuilderByName(int, SerialBaudrate, private);
  /**
   * @brief Set and Get LiDAR Maximum number of abnormal checks.
   * @note When the LiDAR Turn On, if the number of times of abnormal data acquisition
   * is greater than the current AbnormalCheckCount, the LiDAR Fails to Turn On.\n
   * @details The Minimum abnormal value is Two,
   * if it is less than the Minimum Value, it will be set to the Mimimum Value.\n
   * @see CYdLidar::setAbnormalCheckCount and CYdLidar::getAbnormalCheckCount
   */
  PropertyBuilderByName(int, AbnormalCheckCount, private);
  /**
   * @brief Set and Get LiDAR Serial port or network IP address.
   * @note If it is serial port,
   * your need to ensure that the serial port had read and write permissions.\n
   * If it is a network, make sure the network can ping.\n
   * @see CYdLidar::setSerialPort and CYdLidar::getSerialPort
   */
  PropertyBuilderByName(std::string, SerialPort, private);
  /**
   * @brief Set and Get LiDAR  filtering angle area.
   * @note If the LiDAR angle is in the IgnoreArray,
   * the current range will be set to zero.\n
   * Filtering angles need to appear in pairs.\n
   * @details The purpose of the current paramter is to filter out the angular area set by user\n
   * @par example: Filters 10 degrees to 30 degrees and 80 degrees to 90 degrees.
   * @code
   *    CYdLidar laser;//Defining an CYdLidar instance.
   *    std::vector<float> ignore_array;
   *    ignore_array.push_back(10.0);
   *    ignore_array.push_back(30.0);
   *    ignore_array.push_back(80.0);
   *    ignore_array.push_back(90.0);
   *    laser.setIgnoreArray(ignore_array);
   * @endcode
   * @see CYdLidar::setIgnoreArray and CYdLidar::getIgnoreArray
   */
  PropertyBuilderByName(std::vector<float>, IgnoreArray, private);

  PropertyBuilderByName(float, OffsetTime, private);
  /**
   * @brief Set and Get LiDAR single channel.
   * Whether LiDAR communication channel is a single-channel
   * @note For a single-channel LiDAR, if the settings are reversed.\n
   * an error will occur in obtaining device information and the LiDAR will Faied to Start.\n
   * For dual-channel LiDAR, if th setttings are reversed.\n
   * the device information cannot be obtained.\n
   * Set the single channel to match the LiDAR.
   * @remarks
   <table>
        <tr><th>G1/G2/G2A/G2C                          <td>false
        <tr><th>G4/G4B/G4PRO/G6/F4/F4PRO               <td>false
        <tr><th>S4/S4B/X4/R2/G4C                       <td>false
        <tr><th>S2/X2/X2L                              <td>true
        <tr><th>TG15/TG30/TG50                         <td>false
        <tr><th>TX8/TX20                               <td>true
        <tr><th>T5/T15                                 <td>false
        <tr><th>                                       <td>true
    </table>
   * @see CYdLidar::setSingleChannel and CYdLidar::getSingleChannel
   */
  PropertyBuilderByName(bool, SingleChannel, private);
  /**
  * @brief Set and Get LiDAR Type.
  * @note Refer to the table below for the LiDAR Type.\n
  * Set the LiDAR Type to match the LiDAR.
  * @remarks
  <table>
       <tr><th>G1/G2A/G2/G2C                    <td>[TYPE_TRIANGLE](\ref LidarTypeID::TYPE_TRIANGLE)
       <tr><th>G4/G4B/G4C/G4PRO                 <td>[TYPE_TRIANGLE](\ref LidarTypeID::TYPE_TRIANGLE)
       <tr><th>G6/F4/F4PRO                      <td>[TYPE_TRIANGLE](\ref LidarTypeID::TYPE_TRIANGLE)
       <tr><th>S4/S4B/X4/R2/S2/X2/X2L           <td>[TYPE_TRIANGLE](\ref LidarTypeID::TYPE_TRIANGLE)
       <tr><th>TG15/TG30/TG50/TX8/TX20          <td>[TYPE_TOF](\ref LidarTypeID::TYPE_TOF)
       <tr><th>T5/T15                           <td>[TYPE_TOF_NET](\ref LidarTypeID::TYPE_TOF_NET)
   </table>
  * @see [LidarTypeID](\ref LidarTypeID)
  * @see CYdLidar::setLidarType and CYdLidar::getLidarType
  */
  PropertyBuilderByName(int, LidarType, private);

 public:
  CYdLidar(); //!< Constructor
  virtual ~CYdLidar();  //!< Destructor: turns the laser off.
  /*!
   * @brief initialize
   * @return
   */
  bool initialize();  //!< Attempts to connect and turns the laser on. Raises an exception on error.

  // Return true if laser data acquistion succeeds, If it's not
  bool doProcessSimple(LaserScan &outscan,
                       bool &hardwareError);

  //Turn on the motor enable
  bool  turnOn();  //!< See base class docs

  //Turn off the motor enable and close the scan
  bool  turnOff(); //!< See base class docs

  //Turn off lidar connection
  void disconnecting(); //!< Closes the comms with the laser. Shouldn't have to be directly needed by the user

  //get zero angle offset value
  float getAngleOffset() const;

  //Whether the zero offset angle is corrected?
  bool isAngleOffetCorrected() const;

  //! get lidar software version
  std::string getSoftVersion() const;

  //! get lidar hardware version
  std::string getHardwareVersion() const;

  //! get lidar serial number
  std::string getSerialNumber() const;

 protected:
  /*! Returns true if communication has been established with the device. If it's not,
    *  try to create a comms channel.
    * \return false on error.
    */
  bool  checkCOMMs();

  /*! Returns true if health status and device information has been obtained with the device. If it's not,
    * \return false on error.
    */
  bool  checkStatus();

  /*! Returns true if the normal scan runs with the device. If it's not,
    * \return false on error.
    */
  bool checkHardware();

  /*! Returns true if the device is in good health, If it's not*/
  bool getDeviceHealth();

  /*! Returns true if the device information is correct, If it's not*/
  bool getDeviceInfo();

  /*!
   * @brief checkSampleRate
   */
  void checkSampleRate();

  /**
   * @brief CalculateSampleRate
   * @param count
   * @return
   */
  bool CalculateSampleRate(int count, double scan_time);

  /*! Retruns true if the scan frequency is set to user's frequency is successful, If it's not*/
  bool checkScanFrequency();

  /*! returns true if the lidar data is normal, If it's not*/
  bool checkLidarAbnormal();

  /*!
   * @brief checkCalibrationAngle
   * @param serialNumber
   */
  void checkCalibrationAngle(const std::string &serialNumber);

  /*!
    * @brief isRangeValid
    * @param reading
    * @return
    */
  bool isRangeValid(double reading) const;

  /*!
   * @brief isRangeIgnore
   * @param angle
   * @return
   */
  bool isRangeIgnore(double angle) const;

  /*!
   * @brief handleSingleChannelDevice
   */
  void handleSingleChannelDevice();

  /**
   * @brief parsePackageNode
   * @param node
   * @param info
   */
  void parsePackageNode(const node_info &node, LaserDebug &info);

  /**
   * @brief handleDeviceInfoPackage
   * @param count
   */
  void handleDeviceInfoPackage(int count);

  /**
   * @brief printfVersionInfo
   * @param info
   */
  void printfVersionInfo(const device_info &info);

 private:
  bool    isScanning;
  int     m_FixedSize ;
  float   m_AngleOffset;
  bool    m_isAngleOffsetCorrected;
  float   frequencyOffset;
  int   lidar_model;
  uint8_t Major;
  uint8_t Minjor;
  YDlidarDriver *lidarPtr;
  uint64_t m_PointTime;
  uint64_t last_node_time;
  node_info *global_nodes;
  std::map<int, int> SampleRateMap;
  bool m_ParseSuccess;
  std::string m_lidarSoftVer;
  std::string m_lidarHardVer;
  std::string m_lidarSerialNum;
  int defalutSampleRate;
  int m_UserSampleRate;
};	// End of class

