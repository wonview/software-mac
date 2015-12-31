#ifndef WAPI_SMS4_H
#define WAPI_SMS4_H


/** \@{
 * @ingroup crypto
 */
/**
 * @brief SMS4
 *
 * @par Description
 *
 * @param[in]   iv          : Initialization value/integrity vector
 * @param[in]   key         : key
 * @param[in]   input       : input text
 * @param[in]   length      : Length of text
 * @param[inout]output      : Computed sms4 output
 *
 * @return
 *   void
 */
void WapiCryptoSms4(u8 *iv, u8 *key, u8 *input, u16 length, u8 *output);

/** \@{
 * @ingroup crypto
 */
/**
 * @brief SMS4 MIC
 *
 * @par Description
 *
 * @param[in]   iv          : Initialization value/integrity vector
 * @param[in]   key         : key
 * @param[in]   header      : header padded to 16 byte boundary
 * @param[in]   headerLength: Length of header
 * @param[in]   input       : input data
 * @param[in]   length      : Length of data
 * @param[inout]output      : Computed MIC
 *
 * @return
 *   void
 */
void WapiCryptoSms4Mic(u8 *iv, u8 *Key, u8 *header, u16 headerLength,
                             const u8 *input, u16 dataLength, u8 *output);

/** \@}
 */



#endif
