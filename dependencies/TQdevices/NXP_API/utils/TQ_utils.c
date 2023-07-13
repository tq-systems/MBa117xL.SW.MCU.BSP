//******************************************************************************
/*!
 * \copyright
 * SPDX-License-Identifier: BSD-3-Clause
 * \copyright
 * Copyright (c) 2021 - 2023 TQ-Systems GmbH <license@tq-group.com>,
 * D-82229 Seefeld, Germany.
 * Author: Isaac L. L. Yuki
 */
//******************************************************************************

/*!
 * \addtogroup TQ_utils
 * @{
 * \file
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "TQ_utils.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * \brief Reads a line of text from standard input.
 *
 * This function reads characters from standard input until a newline character
 * (either CR or LF) is encountered or until the buffer is one character away
 * from being full. It handles backspace (BS) characters by moving the cursor
 * back one position if possible, effectively deleting the last character
 * entered. Each character entered by the user, except BS, is echoed back via
 * PRINTF.
 * \param[out] buffer A pointer to the character array to store the input
 * string. The buffer is zero-initialized before characters are read.
 * \param[in] bufferSize The total size of the buffer. The function will stop
 * reading one character before this size is reached to leave space for the null
 * terminator.
 * \param[out] actualSize A pointer to a size_t variable where the function will
 * store the actual number of characters read (excluding the null terminator).
 * If the input is longer than the buffer can accommodate, this equals
 * bufferSize.
 * \return True if the function successfully reads a line of input;
 * false if an overflow attempt occurs when the input string exceeds the buffer
 * capacity.
 * \note The input through the terminal has to be terminated with
 * either CR or LF.
 * \note The input string will always be null-terminated.
 * \note
 * This function echoes input characters back to the user using PRINTF.
 * Sensitive input, such as passwords, should be handled with a different
 * mechanism that does not echo the input.
 */
bool getline(char *buffer, size_t bufferSize, size_t *actualSize)
{
  char   input = '\0';
  size_t i     = 0;
  memset(buffer, 0, bufferSize);
  *actualSize = 0;
  while (true)
  {
    input = GETCHAR();
    if (input == '\r' || input == '\n')
    {
      break;
    }
    else if (input == '\b')
    {
      if (i > 0)
      {
         /*Erase last char on console: 
         * Move cursor back one space (\b), overwrite with space (' '), 
         * and then move back again (\b) to position the cursor correctly.*/
        PRINTF("\b \b");
        i--;
      }
      buffer[i] = '\0';
    }
    else if (i < bufferSize - 1)
    {
      PRINTF("%c", input);
      buffer[i] = input;
      i++;
    }
    else
    {
      PRINTF("\r\n");
      buffer[i] = '\0';
      if (actualSize)
      {
        *actualSize = i;
      }
      return false;
    }
  }
  PRINTF("\r\n");
  buffer[i] = '\0';
  if (actualSize)
  {
    *actualSize = i;
  }
  return true;
}

/*!@}*/
