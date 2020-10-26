#include "security_adapter.h"
#if SECURE == 1
#include "mbedtls/aes.h"
#endif

void security_adapter_encrypt(
	const uint8_t *secure_key,
	uint8_t *encrypted_payload, 
	uint8_t *encrypted_payload_length,
	uint8_t *decrypted_payload,
	uint8_t decrypted_payload_length) 
{
#if SECURE == 1
	uint16_t i;
	mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_enc(&aes, (const unsigned char*) secure_key, SECURITY_KEY_SIZE*8);
	
	for (i = 0; i < decrypted_payload_length; i+= SECURITY_KEY_SIZE) {
        mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, &decrypted_payload[i], &encrypted_payload[i]);
	}

	*encrypted_payload_length = i;

    mbedtls_aes_free(&aes);
#endif
}

void security_adapter_decrypt(
	const uint8_t *secure_key,
	uint8_t *encrypted_payload, 
	uint8_t encrypted_payload_length,
	uint8_t *decrypted_payload,
	uint8_t *decrypted_payload_length)
{
#if SECURE == 1
	// assert(encrypted_payload_length % SECURITY_KEY_SIZE == 0);	

	uint16_t i;
	mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_enc(&aes, (const unsigned char*) secure_key, SECURITY_KEY_SIZE*8);
	
	for (i = 0; i < encrypted_payload_length; i+= SECURITY_KEY_SIZE) {
        mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT, &encrypted_payload[i], &decrypted_payload[i]);
	}

	*decrypted_payload_length = i;
    
    mbedtls_aes_free(&aes);
#endif
}
