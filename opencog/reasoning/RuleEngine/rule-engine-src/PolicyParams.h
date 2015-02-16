/*
 * PolicyParam.h
 *
 * Copyright (C) 2014 Misgana Bayetta
 *
 * Author: Misgana Bayetta <misgana.bayetta@gmail.com>   2015
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef POLICYPARAMS_H_
#define POLICYPARAMS_H_

#include <string>
using namespace std;

//json object key names
const string RULES = "rules";
const string FILE_PATH = "file_path";
const string RULE_NAME = "name";
const string PRIORITY = "priority";
const string CATEGORY = "category";
const string ATTENTION_ALLOC = "attention_allocation";
const string LOG_LEVEL = "log_level";
const string MUTEX_RULES = "mutex";
const string MAX_ITER = "max_iter";

#endif /* POLICYPARAMS_H_ */
