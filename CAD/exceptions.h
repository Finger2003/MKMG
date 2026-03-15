#pragma once

#define WIDEN2(x) L ## x
#define WIDEN(x) WIDEN2(x)
#define __WFILE__ WIDEN(__FILE__)

#define STRINGIFY(x) #x
#define TOWSTRING(x) WIDEN(STRINGIFY(x))
#define __AT__ __WFILE__ L":" TOWSTRING(__LINE__)

class Exception
{
public:
	virtual ~Exception() = default;

	/**
	* @brief Creates new Exception with the given location description.
	* @param [in] location Wide C string describing location of the error.
	*/
	explicit Exception(const wchar_t* location) : m_location(location) {}

	/**
	* @brief Gets the error message associated with this exception.
	* @returns A wide string describing the error and its location.
	*/
	virtual std::wstring getMessage() const = 0;

	/**
	* @brief Gets the error code associated with this exception.
	* @returns Code number of the error.
	*/
	virtual int getExitCode() const = 0;

	/**
	* Gets the location of the error.
	* @returns Wide C string describing location of the error.
	*/
	const wchar_t* getErrorLocation() const { return m_location; }
private:
	const wchar_t* m_location; // Location of the error
};

/**
* * @brief Exception representing an error encountered by a system function call.
*/
class WinAPIException : public Exception
{
public:
	/*
	* Creates a new exception with the given location and eror code
	* @param [in] location Wide C string describing location of the error.
	* @param [in] errorCode Error code associated with the exception. This should be one of System Error Codes
	*/
	explicit WinAPIException(const wchar_t* location, DWORD errorCode = GetLastError());

	/*
	* Gets the error code associated with this exception.
	* @returns Code number of the error.
	*/
	int getExitCode() const override { return static_cast<int>(m_code); }

	/*
	* Gets the error code associated with this exception.
	* @returns Code number of the error.
	*/
	DWORD getErrorCode() const { return m_code; }

	/*
	* Gets the error message associated with this exception.
	* @returns String describing the error and its location.
	* @remark The error message is obtained based on the error code using FormatMessage function.
	*/
	std::wstring getMessage() const override;

private:
	DWORD m_code; // Error code associated with the exception
};



/**
 * @brief Exception class for storing error location and custom error message.
 */
class CustomException : public Exception
{
public:
	/**
	 * @brief Creates a new exception with the given location and custom error message.
	 *
	 * @param location Wide C string describing location of the error.
	 * @param message Wide string custom error message describing the error.
	 */
	CustomException(const wchar_t* location, const std::wstring& message);

	/**
	 * @brief Creates a new exception with the given location and custom error message.
	 *
	 * @param location Wide C string describing location of the error.
	 * @param message Wide string custom error message describing the error.
	 */
	CustomException(const wchar_t* location, std::wstring&& message);

	/**
	 * @brief Gets the error code associated with this exception.
	 *
	 * @return Code number of this exception. Since this is a custom exception, it returns -1 to indicate that there is no specific error code.
	 */
	int getExitCode() const override { return -1; }

	/**
	 * @brief Gets the error message associated with this exception.
	 *
	 * @return A wide string describing the error and its location.
	 */
	std::wstring getMessage() const override;

private:
	std::wstring m_message; // Custom error message
};
#define THROW_WINAPI throw WinAPIException(__AT__)
#define THROW(msg) throw CustomException(__AT__, msg)