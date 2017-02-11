/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frclib;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import hallib.HalDashboard;
import trclib.TrcDbgTrace;

/**
 * This class implements a choice menu where a number of choices are presented to the user on the dashboard. The user
 * can make the selection.
 */
public class FrcChoiceMenu<T>
{
    private static final String moduleName = "FrcChoiceMenu";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    /**
     * This class defines a choice item in a choice menu.
     */
    private class ChoiceItem
    {
        private String choiceText;
        private T choiceObject;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param choiceText specifies the text to be displayed in the choice menu.
         * @param choiceObject specifies the object to be returned if the choice is selected.
         */
        public ChoiceItem(String choiceText, T choiceObject)
        {
            this.choiceText = choiceText;
            this.choiceObject = choiceObject;
        }   //ChoiceItem

        /**
         * This method returns the choice text.
         *
         * @return choice text.
         */
        public String getText()
        {
            return choiceText;
        }   //getText;

        /**
         * This method returns the choice object.
         *
         * @return choice object.
         */
        public T getObject()
        {
            return choiceObject;
        }   //getObject

    }   //class ChoiceItem

    private final String menuTitle;
    private SendableChooser<T> chooser;
    private ArrayList<ChoiceItem> choiceItems;
    private ChoiceItem defChoice = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param menuTitle specifies the title of the menu.
     */
    public FrcChoiceMenu(final String menuTitle)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + menuTitle, tracingEnabled, traceLevel, msgLevel);
        }

        if (menuTitle == null)
        {
            throw new NullPointerException("menuTitle cannot be null.");
        }

        this.menuTitle = menuTitle;
        chooser = new SendableChooser<>();
        choiceItems = new ArrayList<>();
        HalDashboard.putData(menuTitle, chooser);
    }   //FrcChoiceMenu

    /**
     * This method returns the title text of this menu.
     *
     * @return title text.
     */
    public String getTitle()
    {
        final String funcName = "getTitle";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", menuTitle);
        }

        return menuTitle;
    }   //getTitle

    /**
     * This method adds a choice to the menu. The choices will be displayed in the order of them being added.
     *
     * @param choiceText specifies the choice text that will be displayed on the dashboard.
     * @param choiceObject specifies the object to be returned if the choice is selected.
     */
    public void addChoice(String choiceText, T choiceObject)
    {
        final String funcName = "addChoice";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "text=%s,obj=%s",
                choiceText, choiceObject.toString());
        }

        ChoiceItem choiceItem = new ChoiceItem(choiceText, choiceObject);
        choiceItems.add(choiceItem);
        if (defChoice == null)
        {
            //
            // This is the first added choice in the menu. Make it the default choice.
            //
            chooser.addDefault(choiceText, choiceObject);
            defChoice = choiceItem;
        }
        else
        {
            chooser.addObject(choiceText, choiceObject);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //addChoice

    /**
     * This method returns the current selected choice item. Every menu has a current choice even if the menu hasn't
     * been displayed and the user hasn't picked a choice. In that case, the current choice is the default selection
     * of the menu which is the first choice in the menu. If the menu is empty, the current choice is null.
     *
     * @return current selected choice, null if menu is empty.
     */
    public ChoiceItem getCurrentChoice()
    {
        final String funcName = "getCurrentChoice";
        ChoiceItem currChoice = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        T choiceObject = chooser.getSelected();
        for (ChoiceItem choiceItem: choiceItems)
        {
            if (choiceItem.getObject() == choiceObject)
            {
                currChoice = choiceItem;
                break;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", currChoice.getText());
        }

        return currChoice;
    }   //getCurrentChoice

    /**
     * This method returns the text of the current choice. Every menu has a current choice even if the menu hasn't
     * been displayed and the user hasn't picked a choice. In that case, the current choice is the default selection
     * of the menu which is the first choice in the menu. If the menu is empty, the current choice is null.
     *
     * @return current selected choice text, null if menu is empty.
     */
    public String getCurrentChoiceText()
    {
        ChoiceItem choiceItem = getCurrentChoice();
        return choiceItem != null? choiceItem.getText(): null;
    }   //getCurrentChoiceText

    /**
     * This method returns the object of the current choice. Every menu has a current choice even if the menu hasn't
     * been displayed and the user hasn't picked a choice. In that case, the current choice is the default selection
     * of the menu which is the first choice in the menu. If the menu is empty, the current choice is null.
     *
     * @return current choice object, null if menu is empty.
     */
    public T getCurrentChoiceObject()
    {
        ChoiceItem choiceItem = getCurrentChoice();
        return choiceItem != null? choiceItem.getObject(): null;
    }   //getCurrentChoiceObject

}   //class FrcChoiceMenu
