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
 * A plain Java bean to hold configuration of the Link Grammar parser.
 * Some configuration parameters are not really passed onto the parser,
 * but applied only when constructing a <code>ParseResult</code>. This
 * is currently only <code>maxLinkages</code>.
 *
 * @author Borislav Iordanov
 */
public class LGConfig
{
	private int maxLinkages = 25;
	private int maxParseSeconds = 60;
	private double maxCost = -1.0;
	private boolean storeConstituentString = false;
	private boolean storeDiagramString = false;
	private boolean storeSense = false;

	public int getMaxLinkages()
	{
		return maxLinkages;
	}
	public void setMaxLinkages(int m)
	{
		maxLinkages = m;
	}
	public int getMaxParseSeconds()
	{
		return maxParseSeconds;
	}
	public void setMaxParseSeconds(int m)
	{
		maxParseSeconds = m;
	}
	public double getMaxCost()
	{
		return maxCost;
	}
	public void setMaxCost(double m)
	{
		maxCost = m;
	}
	public boolean isStoreConstituentString()
	{
		return storeConstituentString;
	}
	public void setStoreConstituentString(boolean s)
	{
		storeConstituentString = s;
	}

	public boolean isStoreDiagramString()
	{
		return storeDiagramString;
	}
	public void setStoreDiagramString(boolean s)
	{
		storeDiagramString = s;
	}

	public boolean isStoreSense()
	{
		return storeSense;
	}
	public void setStoreSense(boolean s)
	{
		storeSense = s;
	}
}
