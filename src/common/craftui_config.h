/*
   * (C) 2015, Aurel Wildfellner
   *
   * This file is part of CraftUI.
   *
   * Beholder is free software: you can redistribute it and/or modify
   * it under the terms of the GNU General Public License as published by
   * the Free Software Foundation, either version 3 of the License, or
   * (at your option) any later version.
   *
   * Beholder is distributed in the hope that it will be useful,
   * but WITHOUT ANY WARRANTY; without even the implied warranty of
   * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   * GNU General Public License for more details.
   *
   * You should have received a copy of the GNU General Public License
   * along with CraftUI. If not, see <http://www.gnu.org/licenses/>. */

#define CRAFTUI_VERSION 0.1

#define CRAFTUI_DEBUG_LVL 4
#if CRAFTUI_DEBUG_LVL > 0
    #define CRAFTUI_INFO
#endif
#if CRAFTUI_DEBUG_LVL > 1
    #define CRAFTUI_LOG
#endif
#if CRAFTUI_DEBUG_LVL > 2
    #define CRAFTUI_VERBOSE
#endif
# if CRAFTUI_DEBUG_LVL > 3
    #define CRAFTUI_FLOOD
#endif

