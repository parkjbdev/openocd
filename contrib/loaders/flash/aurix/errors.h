/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2026 Infineon Technologies AG
 */
#ifndef ERRORS_H
#define ERRORS_H

#ifndef BIT
#define BIT(nr) (1u << (nr))
#endif

#define ERROR_BUSY BIT(31)
#define ERROR_TIMEOUT BIT(30)
#define ERROR_PAGE BIT(29)
#endif /* ERRORS_H */
