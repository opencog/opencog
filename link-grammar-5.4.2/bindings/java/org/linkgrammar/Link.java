
/*************************************************************************/
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

package org.linkgrammar;

/**
 * Represents a <em>Link Grammar</em> link as part of a parse result.
 * Link objects contain the indices of the word they link to as well
 * as an identifier of the type of link and label for the left and
 * right connectors.
 *
 * @author Borislav Iordanov
 *
 */
public class Link
{
	private int left, right;
	private String label, leftLabel, rightLabel;

	public int getLeft()
	{
		return left;
	}
	public void setLeft(int left)
	{
		this.left = left;
	}
	public int getRight()
	{
		return right;
	}
	public void setRight(int right)
	{
		this.right = right;
	}
	public String getLabel()
	{
		return label;
	}
	public void setLabel(String label)
	{
		this.label = label;
	}
	public String getLeftLabel()
	{
		return leftLabel;
	}
	public void setLeftLabel(String leftLabel)
	{
		this.leftLabel = leftLabel;
	}
	public String getRightLabel()
	{
		return rightLabel;
	}
	public void setRightLabel(String rightLabel)
	{
		this.rightLabel = rightLabel;
	}
}
